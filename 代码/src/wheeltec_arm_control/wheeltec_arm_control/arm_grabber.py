#!/usr/bin/env python3
# coding=utf-8

import json
import os
import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
import threading
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String


ARM_OFFSETS_FILE = os.environ.get(
    "ROBOT_ARM_OFFSETS_FILE",
    "/home/zyc/robot2/config/arm_grab_offsets.json",
)

class ArmGrabberNode(Node):
    def __init__(self):
        super().__init__('arm_grabber_node')
        
        # ==========================================
        # 1. 串口初始化
        # ==========================================
        self.port_name = '/dev/wheeltec_arm' 
        self.baud_rate = 115200
        self.serial_lock = threading.Lock()
        self.abort_event = threading.Event()
        self.is_busy = False             # 正在执行动作，防止重复触发
        self.last_pose = [0.0, 1.0, -1.57, -1.57, 0.0, 0.0]
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=0.5)
            self.get_logger().info(f"🔥 成功连接 STM32 串口: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            self.serial_port = None
            return

        # 给单片机串口芯片 1.0 秒的稳定电平时间，防止开机数据丢包
        self.get_logger().info("等待串口硬件接收器整备...")
        time.sleep(1.0)

        # 发送开机回正指令 (初始化归位姿态)
        self.get_logger().info("正在向单片机发射初始化归位密令...")
        self.send_joint_angles(0.0, 1.0, -1.57, -1.57, 0.0, 0.0, mode=2)

        # 订阅双目视觉话题
        self.subscription = self.create_subscription(
            PointStamped,       
            '/target_point',    
            self.target_callback,
            10)
        self.cmd_sub = self.create_subscription(
            String,
            '/arm_cmd',
            self.arm_cmd_callback,
            10)
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.teleop_sub = self.create_subscription(
            JointState,
            'arm_teleop',
            self.arm_teleop_callback,
            10)
        self.status_pub = self.create_publisher(String, '/arm_status', 10)

        # ==========================================
        # 2. ⚡ 指挥官精测物理尺寸参数 (轴心到轴心)
        # ==========================================
        self.link_a = 0.105           # 大臂轴距: 10.5cm
        self.link_c = 0.100           # 小臂轴距: 10.0cm
        self.link_gripper = 0.150     # J4轴心到两片夹爪闭合中心的净长度: 15.0cm
        
        # ==========================================
        # 🎯 核心物理标定区 (立体空间外参对齐)
        # ==========================================
        self.measured_cam_z = 0.32          
        self.measured_horizontal_y = 0.28   # 28cm 水平距离
        self.camera_offset_y = -0.08  # 底座到相机真实水平距离: 8cm
        self.camera_offset_z = 0.30   # 相机距离底座垂直高度: 30cm

        # 🔍 毫米级极细偏置修正
        self.x_offset =  -0.07   
        self.y_offset = -0.05
        self.z_offset = 0.05   # 高度 1.5cm 降落修正
        self._load_persisted_offsets()

        self.get_logger().info(f"🤖 动力学平稳时间轴网络已建立！稳定压倒一切。")

        # ==========================================
        # 🎯 多帧平均累积区
        # ==========================================
        self.samples_needed = 5          # 收集几张图的坐标
        self.collected_x = []            # 累积 x 坐标
        self.collected_y = []            # 累积 y 坐标
        self.collected_z = []            # 累积 z 坐标
        self.has_executed = False        # 抓取完成标志位
        self.grab_enabled = False        # 触摸屏/手机确认后才抓取
        self.right_turn_rad = 0.75       # 抓取后底座向右转角度，方向反了就改成 -0.75
        self.preview_detection_count = 0
        self.preview_detection_notified = False
        self.preview_last_seen = 0.0
        self.preview_max_gap = 0.7        # 两帧间隔过大则不算连续识别
        self.preview_presence_state = "unknown"
        self.preview_presence_timer = self.create_timer(
            0.2, self.check_preview_presence
        )

        self.get_logger().info(
            f"机械臂抓取偏移：X={self.x_offset:.3f}, "
            f"Y={self.y_offset:.3f}, Z={self.z_offset:.3f} m"
        )

    @staticmethod
    def _valid_offsets(values):
        return all(
            math.isfinite(value) and -0.5 <= value <= 0.5
            for value in values
        )

    def _load_persisted_offsets(self):
        try:
            with open(ARM_OFFSETS_FILE, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            values = tuple(
                float(data[key])
                for key in ("x_offset", "y_offset", "z_offset")
            )
            if not self._valid_offsets(values):
                raise ValueError("偏移超出 -0.5 到 0.5 米范围")
            self.x_offset, self.y_offset, self.z_offset = values
            self.get_logger().info(
                f"已加载持久化抓取偏移: {ARM_OFFSETS_FILE}"
            )
        except FileNotFoundError:
            self.get_logger().info("尚无持久化抓取偏移，使用程序默认值")
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"读取持久化抓取偏移失败，使用程序默认值: {exc}"
            )

    def _save_persisted_offsets(self):
        data = {
            "x_offset": self.x_offset,
            "y_offset": self.y_offset,
            "z_offset": self.z_offset,
        }
        temp_file = f"{ARM_OFFSETS_FILE}.tmp.{os.getpid()}"
        try:
            os.makedirs(os.path.dirname(ARM_OFFSETS_FILE), exist_ok=True)
            with open(temp_file, "w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_file, ARM_OFFSETS_FILE)
            return True
        except OSError as exc:
            self.get_logger().error(f"持久化抓取偏移失败: {exc}")
            try:
                os.unlink(temp_file)
            except OSError:
                pass
            return False

    def publish_status(self, text):
        self.get_logger().info(text)
        if hasattr(self, "status_pub"):
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    def publish_offset_status(self):
        if not hasattr(self, "status_pub"):
            return
        msg = String()
        msg.data = json.dumps(
            {
                "event": "arm_offsets",
                "x_offset": self.x_offset,
                "y_offset": self.y_offset,
                "z_offset": self.z_offset,
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)

    def check_preview_presence(self):
        """Clear the retained phone hint shortly after the bottle disappears."""
        if self.preview_presence_state == "unknown":
            # Publish after ROS discovery has had time to connect the bridge;
            # this also clears a retained detection from an earlier run.
            self.preview_presence_state = "absent"
            self.publish_status("未发现水瓶")
            return

        bottle_is_stale = (
            self.preview_presence_state == "present"
            and (
                self.grab_enabled
                or time.monotonic() - self.preview_last_seen
                > self.preview_max_gap
            )
        )
        if not bottle_is_stale:
            return

        self.preview_presence_state = "absent"
        self.preview_detection_count = 0
        self.preview_detection_notified = False
        self.publish_status("未发现水瓶")

    def arm_cmd_callback(self, msg):
        raw_cmd = msg.data.strip()
        try:
            payload = json.loads(raw_cmd)
        except (json.JSONDecodeError, TypeError):
            payload = None
        if isinstance(payload, dict) and str(payload.get('type', '')).lower() == 'set_offsets':
            self._set_grab_offsets(payload)
            return

        cmd = raw_cmd.upper()
        if cmd in ("GRAB_BOTTLE", "GRAB"):
            if self.is_busy:
                self.publish_status("机械臂正在执行抓取流程，请等待完成；重复抓取指令已忽略。")
                return
            self.abort_event.clear()
            self.has_executed = False
            self.grab_enabled = True
            self.collected_x.clear()
            self.collected_y.clear()
            self.collected_z.clear()
            self.publish_status("收到抓取指令：开始收集双目目标坐标。")
        elif cmd == "SCAN":
            self.has_executed = False
            self.grab_enabled = False
            self.collected_x.clear()
            self.collected_y.clear()
            self.collected_z.clear()
            self.publish_status("收到扫描指令：请确认 YOLO 节点正在输出目标画面，然后点击“抓取水瓶”。")
        elif cmd == "RESET":
            self.abort_event.set()
            self.has_executed = False
            self.grab_enabled = False
            self.collected_x.clear()
            self.collected_y.clear()
            self.collected_z.clear()
            self.publish_status("收到复位指令：机械臂回到初始姿态。")
            self.send_joint_angles(0.0, 1.0, -1.57, -1.57, 0.0, 0.0, mode=2)
        elif cmd == "STOP":
            self.abort_event.set()
            self.grab_enabled = False
            self.publish_status("收到停止指令：停止接收新的抓取任务，机械臂保持当前位置。")
        elif cmd in ("RELEASE", "OPEN", "OPEN_GRIPPER"):
            self.release_gripper()
        elif cmd in ("CLOSE", "CLOSE_GRIPPER"):
            self.close_gripper()

    def _set_grab_offsets(self, payload):
        if self.is_busy:
            self.publish_status("机械臂正在抓取，请在本次动作完成后再调整偏移。")
            return
        try:
            x_offset = float(payload['x_offset'])
            y_offset = float(payload['y_offset'])
            z_offset = float(payload['z_offset'])
        except (KeyError, TypeError, ValueError):
            self.publish_status("抓取偏移设置失败：X、Y、Z 必须是有效数字。")
            return
        offsets = (x_offset, y_offset, z_offset)
        if not all(math.isfinite(value) and -0.5 <= value <= 0.5 for value in offsets):
            self.publish_status("抓取偏移设置失败：每项必须在 -0.5 到 0.5 米之间。")
            return
        self.x_offset, self.y_offset, self.z_offset = offsets
        saved = self._save_persisted_offsets()
        suffix = "，已持久化保存" if saved else "，但持久化保存失败"
        self.publish_status(
            f"抓取偏移已更新：X={self.x_offset:.3f}, "
            f"Y={self.y_offset:.3f}, Z={self.z_offset:.3f} m{suffix}"
        )
        self.publish_offset_status()

    def release_gripper(self):
        if self.is_busy:
            self.publish_status("机械臂正在执行抓取流程，请等待完成后再松开夹爪。")
            return
        pose = list(self.last_pose)
        pose[5] = 1.57
        self.publish_status("收到手机指令：原地张开夹爪，放下水瓶。")
        self.send_joint_angles(*pose, mode=2)

    def close_gripper(self):
        if self.is_busy:
            self.publish_status("机械臂正在执行抓取流程，夹爪闭合指令已忽略。")
            return
        pose = list(self.last_pose)
        pose[5] = -0.4
        self.publish_status("收到手机指令：原地闭合夹爪。")
        self.send_joint_angles(*pose, mode=2)

    def forward_joint_command(self, msg, source):
        if len(msg.position) < 6:
            # 底盘也会发布全局 joint_states，通常只有 2~4 个轮关节。
            # 这不是机械臂错误，静默过滤即可；手机专用 arm_teleop
            # 若数据不完整仍保留提醒。
            if source != 'joint_states':
                self.get_logger().warning(
                    f"忽略 {source} 指令：关节位置少于 6 个。"
                )
            return
        if self.is_busy:
            return
        self.send_joint_angles(*msg.position[:6], mode=1)

    def joint_states_callback(self, msg):
        """转发运动规划控制指令，与视觉抓取共用同一串口。"""
        self.forward_joint_command(msg, 'joint_states')

    def arm_teleop_callback(self, msg):
        """转发遥控/手柄控制指令，与视觉抓取共用同一串口。"""
        self.forward_joint_command(msg, 'arm_teleop')

    def send_joint_angles(self, rad1, rad2, rad3, rad4, rad5, rad6, mode=1):
        """严格大端序打包发送函数"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("❌ 串口未打开，机械臂控制帧没有发送。")
            return

        j1 = int(rad1 * 1000)
        j2 = int(rad2 * 1000)
        j3 = int(rad3 * 1000)
        j4 = int(rad4 * 1000)
        j5 = int(rad5 * 1000)
        j6 = int(rad6 * 1000)

        data = bytearray(16)
        data[0] = 0xAA  
        
        struct.pack_into('>h', data, 1, j1)
        struct.pack_into('>h', data, 3, j2)
        struct.pack_into('>h', data, 5, j3)
        struct.pack_into('>h', data, 7, j4)
        struct.pack_into('>h', data, 9, j5)
        struct.pack_into('>h', data, 11, j6)
        
        data[13] = mode 

        check_sum = 0
        for i in range(14):
            check_sum ^= data[i]
        data[14] = check_sum
        data[15] = 0xBB 

        try:
            with self.serial_lock:
                total = 0
                # STM32/USB CDC 偶尔会漏掉单帧，演示动作每次重复下发 3 次更稳。
                for _ in range(3):
                    total += self.serial_port.write(data)
                    self.serial_port.flush()
                    time.sleep(0.03)
            self.last_pose = [rad1, rad2, rad3, rad4, rad5, rad6]
            self.get_logger().info(
                f"串口下发成功: J1={rad1:.3f}, J2={rad2:.3f}, J3={rad3:.3f}, "
                f"J4={rad4:.3f}, J5={rad5:.3f}, J6={rad6:.3f}, mode={mode}, bytes={total}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ 串口数据发射异常: {e}")

    def wait_or_abort(self, seconds):
        return self.abort_event.wait(seconds)

    def target_callback(self, msg):
        if not self.grab_enabled:
            now = time.monotonic()
            if now - self.preview_last_seen > self.preview_max_gap:
                self.preview_detection_count = 0
                self.preview_detection_notified = False
            self.preview_last_seen = now
            self.preview_detection_count += 1
            if (
                self.preview_detection_count >= 2
                and not self.preview_detection_notified
            ):
                self.preview_detection_notified = True
                self.preview_presence_state = "present"
                self.publish_status("发现水瓶")
            return

        # 🛡️ 已经完成抓取，忽略后续所有坐标
        if self.has_executed:
            return

        cam_x = msg.point.x  
        cam_z = msg.point.z  

        # 🎯 收集坐标样本
        self.collected_x.append(cam_x)
        self.collected_y.append(0.0)      # 原逻辑只用 cam_x 和 cam_z，y 为占位
        self.collected_z.append(cam_z)

        if len(self.collected_x) == 1:
            self.publish_status(
                "已识别到瓶子，正在连续确认双目坐标（1/"
                f"{self.samples_needed} 帧）。"
            )

        # ⏳ 样本数不足，继续等待
        if len(self.collected_x) < self.samples_needed:
            if len(self.collected_x) > 1:
                self.publish_status(
                    f"正在收集双目坐标：{len(self.collected_x)}/"
                    f"{self.samples_needed} 帧..."
                )
            return

        # ✅ 样本充足，计算平均值
        avg_x = sum(self.collected_x) / len(self.collected_x)
        avg_z = sum(self.collected_z) / len(self.collected_z)
        self.publish_status(f"目标坐标平均完成：x={avg_x:.3f} m，z={avg_z:.3f} m，共 {len(self.collected_x)} 帧。")

        # 用平均值覆盖 cam_x、cam_z，走后续逻辑
        cam_x = avg_x
        cam_z = avg_z

        # ==========================================
        # 📛 标记已执行，此后消息一律忽略
        # ==========================================
        self.has_executed = True
        self.grab_enabled = False

        # 1. 空间几何投影
        cos_pitch = self.measured_horizontal_y / self.measured_cam_z
        horizontal_depth = cam_z * cos_pitch 
        
        # 2. 转换至机械臂底座坐标系
        bottle_target_y = horizontal_depth + self.camera_offset_y + self.y_offset
        bottle_target_x = cam_x + self.x_offset

        # 3. 实时推算瓶子原本的 3D 空间高度
        sin_pitch = math.sqrt(1 - cos_pitch**2)
        vertical_drop = cam_z * sin_pitch  
        bottle_target_z = self.camera_offset_z - vertical_drop 

        # 4. 🎯 【TCP剥离与真实高度沉降算法】
        Y_wrist = bottle_target_y - self.link_gripper    
        Z_wrist = bottle_target_z + self.z_offset                       

        # 5. 纯几何余弦定理解算大臂与小臂
        D2 = Y_wrist**2 + Z_wrist**2
        cos_j3_raw = (D2 - self.link_a**2 - self.link_c**2) / (2 * self.link_a * self.link_c)
        cos_j3_raw = max(-1.0, min(1.0, cos_j3_raw))
        j3_angle = math.acos(cos_j3_raw)  

        psi = math.atan2(Y_wrist, Z_wrist)
        cos_mu = (self.link_a**2 + D2 - self.link_c**2) / (2 * self.link_a * math.sqrt(D2))
        cos_mu = max(-1.0, min(1.0, cos_mu))
        mu = math.acos(cos_mu)

        # ====================================================
        # ⚠️ 姿态方向合成 (负数向前低头)
        # ====================================================
        j1 = math.atan2(bottle_target_x, bottle_target_y) if bottle_target_y != 0 else 0.0
        
        # 拱起折叠构型
        j2 = -(psi - mu)                  
        j3 = -j3_angle                    
        
        # 🌟 绝对水平锁死公式
        j4 = -1.57 - j2 - j3               
        j5 = 0.0

        # ====================================================
        # 🛡️ 镜像 STM32 control.c 硬限幅安全拦截
        # ====================================================
        if j1 < -1.57 or j1 > 1.57: j1 = max(-1.57, min(1.57, j1))
        if j2 < -1.57 or j2 > 1.57: j2 = max(-1.57, min(1.57, j2))
        if j3 < -1.57 or j3 > 1.57: j3 = max(-1.57, min(1.57, j3))
        if j4 < -0.45 or j4 > 1.57: j4 = max(-0.45, min(1.57, j4))
        
        # ====================================================
        # 🎯 黄金动态时序抓取流水线 (核心微调区)
        # ====================================================
        self.publish_status(f"逆运动学解算完成：J1={j1:.3f}, J2={j2:.3f}, J3={j3:.3f}, J4={j4:.3f}")
        threading.Thread(
            target=self.execute_grab_sequence,
            args=(j1, j2, j3, j4, j5),
            daemon=True,
        ).start()

    def execute_grab_sequence(self, j1, j2, j3, j4, j5):
        self.is_busy = True
        try:
            if self.abort_event.is_set():
                self.publish_status("抓取流程已取消。")
                return

            self.publish_status("动作1：机械臂抬高，移动到水杯上方。")
            self.send_joint_angles(j1, j2 + 0.06, j3, j4 - 0.06, j5, 1.57, mode=2)
            if self.wait_or_abort(2.0): return

            self.publish_status("动作2：向下靠近水杯，等待机械臂稳定。")
            self.send_joint_angles(j1, j2, j3, j4, j5, 1.57, mode=2)
            if self.wait_or_abort(3.0): return

            self.publish_status("动作3：夹爪闭合，抓取水杯。")
            self.send_joint_angles(j1, j2, j3, j4, j5, -0.4, mode=2)
            if self.wait_or_abort(1.2): return

            self.publish_status("动作4：抓取成功，抬起水杯。")
            self.send_joint_angles(j1, j2 + 0.3, j3, j4 - 0.3, j5, -0.4, mode=2)
            if self.wait_or_abort(2.0): return

            # 正右侧展示位：不再叠加当前识别角度，保证每次都转到稳定的右侧方向。
            right_j1 = max(-1.20, min(1.20, self.right_turn_rad))
            self.publish_status(f"动作5：机械臂夹着水杯转到正右侧展示位，J1={right_j1:.3f} rad。")
            self.send_joint_angles(right_j1, j2 + 0.3, j3, j4 - 0.3, j5, -0.4, mode=2)
            if self.wait_or_abort(2.0): return

            self.publish_status("动作6：保持夹爪夹紧，手臂向内回收，方便下一次抓取准备。")
            self.send_joint_angles(right_j1, 0.45, -1.10, -0.35, 0.0, -0.4, mode=2)
            if self.wait_or_abort(1.8): return

            self.publish_status("演示完成：机械臂夹着水杯停在右侧回收姿态，可复位后进行下一次抓取。")
        finally:
            if self.abort_event.is_set():
                self.publish_status("抓取流程已停止，机械臂保持当前位置。")
            self.is_busy = False
            self.grab_enabled = False

def main(args=None):
    rclpy.init(args=args)
    node = ArmGrabberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
