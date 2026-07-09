#!/usr/bin/env python3
"""
WHEELTEC 六轴机械臂 ROS 2 双目视觉抓取节点 (指挥官纯代数投影终极版)
特性:
  - 彻底停用易冲突的 TF 监听器，改用 100% 稳定的高精度三角函数空间投影
  - 引入完整的开机稳定、强力模式、串口全冲刷和防放电闪退抬升机制
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import serial

from .ik_solver import ArmIKSolver
from .serial_protocol import (
    build_serial_frame,
    format_frame_hex,
    MODE_DEFAULT,
    MODE_FOLLOWER,
)
from .grasp_sequencer import GraspSequencer, GraspState


class ArmGraspNode(Node):
    """机械臂抓取主节点"""

    def __init__(self):
        super().__init__('wheeltec_arm_grasp_node')

        # ==========================================
        # 1. 声明并读取参数
        # ==========================================
        self._declare_params()

        # ==========================================
        # 2. 初始化 IK 求解器
        # ==========================================
        self.ik_solver = ArmIKSolver(
            link_a=self.get_parameter('link_a').value,
            link_b=self.get_parameter('link_b').value,
            link_c=self.get_parameter('link_c').value,
            link_h=self.get_parameter('link_h').value,
            auxiliary_angle=self.get_parameter('auxiliary_angle').value,
        )

        ws = self.ik_solver.get_workspace_info()
        self.get_logger().info(
            f'🔧 核心数学解算器已整备: '
            f'工作范围 {ws["min_reach_m"]:.3f}~{ws["max_reach_m"]:.3f}m'
        )

        # ==========================================
        # 3. 打开串口并注入强力冲刷
        # ==========================================
        self._serial_lock = threading.Lock()
        self.serial_port = None
        self._open_serial()

        # ==========================================
        # 4. 多帧平均缓冲区
        # ==========================================
        self._target_buffer = []
        self._grasp_triggered = False   

        # ==========================================
        # 5. 初始化抓取序列器 (激活物理高度垂直抬升)
        # ==========================================
        self.grasper = GraspSequencer(
            ik_solver=self.ik_solver,
            serial_writer=self._send_joints,
            node=self,
            approach_height_z=self.get_parameter('approach_height_z').value,
            lift_height_z=self.get_parameter('lift_height_z').value,
            hand_open_angle=self.get_parameter('hand_open_angle').value,
            hand_close_angle=self.get_parameter('hand_close_angle').value,
            down_offset=self.get_parameter('down_offset').value,
            step_delay=2.0,
        )

        # ==========================================
        # 6. 订阅相机原生态点云话题
        # ==========================================
        self.target_sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self._target_callback,
            10,
        )

        # ==========================================
        # 7. 定时器: 驱动抓取状态机
        # ==========================================
        self._grasp_timer = self.create_timer(2.0, self._grasp_step_callback)

        # ==========================================
        # 8. 初始强力归位
        # ==========================================
        self.get_logger().info('⚡ 正在等待单片机串口硬件整备...')
        time.sleep(1.0)

        self._grasping_active = False
        init_pose = [0.0, 1.0, -1.57, -1.57, 0.0, -0.5]
        self.get_logger().info('🏠 发射初始归位命令：前倾低矮姿态')
        self._send_joints(init_pose, mode=MODE_FOLLOWER)
        self.get_logger().info('🤖 智能代数抓取神经网已全线就位！监听中...')

    def _declare_params(self):
        """核心解算与安装参数本"""
        self.declare_parameter('auxiliary_angle', 0.0)
        self.declare_parameter('link_a', 0.105)
        self.declare_parameter('link_b', 0.105)
        self.declare_parameter('link_c', 0.145)
        self.declare_parameter('link_h', 0.080)
        
        # 毫米级极细偏置微调 (先全部归零)
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        
        # 💥 【战略新增】直观的相机物理摆放参数，允许命令行无痛修改！
        self.declare_parameter('camera_pitch_deg', 20.0)  # 相机向下低头的倾角 (度)
        self.declare_parameter('camera_offset_y', -0.20)  # 相机在机械臂后方多少米 (负值=后方)
        self.declare_parameter('camera_offset_z', 0.25)   # 相机中心比机械臂底座高多少米

        self.declare_parameter('serial_port', '/dev/wheeltec_arm')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('buffer_frames', 5)
        
        # 抓取过程参数
        self.declare_parameter('approach_height_z', 0.05) # 瓶子上方5cm预接近
        self.declare_parameter('lift_height_z', 0.10)     # ✊ 核心修改：抓到后垂直拔高 10 厘米！
        self.declare_parameter('hand_open_angle', 1.57)
        self.declare_parameter('hand_close_angle', -0.4)
        self.declare_parameter('down_offset', -2.2)

    def _open_serial(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f'✅ 串口畅通: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'❌ 串口硬伤: {e}')
            self.serial_port = None

    def _send_joints(self, joint_angles, mode=MODE_DEFAULT):
        """线程安全大端序发射，强制全线冲刷物理缓存"""
        deg_str = ', '.join(f'{a*180/math.pi:+.1f}°' for a in joint_angles)
        self.get_logger().info(f'📤 [串口发射] 关节角度: [{deg_str}]')

        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            frame = build_serial_frame(joint_angles, mode)
            with self._serial_lock:
                self.serial_port.write(frame)
                self.serial_port.flush()  # 👈 核心修正：瞬间逼迫数据冲入铜线，不留操作系统缓存！
        except Exception as e:
            self.get_logger().error(f'❌ 串口硬件死线: {e}')

    def _target_callback(self, msg: PointStamped):
        """核心代数几何回调：通过 3D 三角函数强行投影"""
        if self._grasp_triggered or self._grasping_active:
            return

        try:
            cam_x = msg.point.x    # 相机原生：左右横移
            cam_z = msg.point.z    # 相机原生：视线射出深度

            # 读取战略摆放配置
            pitch_deg = self.get_parameter('camera_pitch_deg').value
            cam_off_y = self.get_parameter('camera_offset_y').value
            cam_off_z = self.get_parameter('camera_offset_z').value
            x_offset = self.get_parameter('x_offset').value
            y_offset = self.get_parameter('y_offset').value

            # ==================================================
            # 📐 核心空间代数转换：将斜向下视线完美拍平到水平面 📐
            # ==================================================
            pitch_rad = math.radians(pitch_deg)
            
            # 1. 算出瓶子相对于相机的真实地平线向前推进深度
            horizontal_depth = cam_z * math.cos(pitch_rad)
            
            # 2. 算出瓶子相对于相机的垂直掉落高度
            vertical_drop = cam_z * math.sin(pitch_rad)

            # 3. 完美对齐到机械臂底座中心点
            true_x = cam_x + x_offset                        # 左右横移不变
            true_y = horizontal_depth + cam_off_y + y_offset # 投影前方深度 - 20cm相机后退
            true_z = cam_off_z - vertical_drop               # 相机挂载高度 - 视线掉落

            self.get_logger().info(
                f'✨ [纯数学空间投影成功] 相机原生: (x={cam_x:.3f}, z={cam_z:.3f}) | '
                f'臂座标准系: (左右_x={true_x:.3f}, 前后_y={true_y:.3f}, 高度_z={true_z:.3f})'
            )

            # ---- 进入多帧平均平滑缓冲区 ----
            buffer_max = self.get_parameter('buffer_frames').value
            self._target_buffer.append((true_x, true_y, true_z))

            if len(self._target_buffer) < buffer_max:
                return

            avg_x = sum(b[0] for b in self._target_buffer) / len(self._target_buffer)
            avg_y = sum(b[1] for b in self._target_buffer) / len(self._target_buffer)
            avg_z = sum(b[2] for b in self._target_buffer) / len(self._target_buffer)
            self._target_buffer.clear()

            self.get_logger().info(
                f'🎯 终极突击目标 (3D均值): x={avg_x:.3f}, y={avg_y:.3f}, z={avg_z:.3f}'
            )

            # ---- 检查机械臂肉体可达圈 ----
            reachable, distance = self.ik_solver.check_reachable(avg_x, avg_y)
            if not reachable:
                ws = self.ik_solver.get_workspace_info()
                self.get_logger().warn(f'⚠️ 目标超出工作范围! 计算距离={distance:.3f}m')
                self._grasp_triggered = True
                return

            # ---- 激活高能状态机 ----
            self._grasp_triggered = True
            self._grasping_active = True
            
            # 💡 提示：瓶子在桌面上，其相对于底座表面的Z坐标应该归零。
            # 如果发现机械臂在空气中抓取，可将下面第三个参数直接改为 0.0 固定抓桌面！
            self.grpers_target_z = max(0.0, avg_z) 
            self.grasper.start_grasp(avg_x, avg_y, 0.0)

        except Exception as e:
            self.get_logger().error(f'❌ 神经网执行异常: {e}')

    def _grasp_step_callback(self):
        """驱动状态机，并为垂直抬升注入核心延时"""
        if not self._grasping_active:
            return

        try:
            still_running = self.grasper.step()
            if not still_running:
                self._grasping_active = False
                self.get_logger().info('✅ 物体已成功垂直提起 10 厘米！保留2秒黄金整备时间...')
                # 👈 核心修正：抬起后死等2秒，保证单片机把舵机开过去，防止节点由于瞬间退出关闭串口导致瓶子打滑坠毁！
                time.sleep(2.0)
                raise SystemExit(0)
        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f'❌ 状态机崩溃: {e}')
            self._grasping_active = False

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port is not None:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()