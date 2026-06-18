#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
from geometry_msgs.msg import PointStamped

class ArmGrabberNode(Node):
    def __init__(self):
        super().__init__('arm_grabber_node')
        
        # ==========================================
        # 1. 串口初始化
        # ==========================================
        self.port_name = '/dev/wheeltec_arm' 
        self.baud_rate = 115200
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
        self.x_offset = -0.020   
        self.y_offset = 0.010      
        self.z_offset = -0.025   # 高度 1.5cm 降落修正

        self.get_logger().info(f"🤖 动力学平稳时间轴网络已建立！稳定压倒一切。")

        # ==========================================
        # 🎯 多帧平均累积区
        # ==========================================
        self.samples_needed = 5          # 收集几张图的坐标
        self.collected_x = []            # 累积 x 坐标
        self.collected_y = []            # 累积 y 坐标
        self.collected_z = []            # 累积 z 坐标
        self.has_executed = False        # 抓取完成标志位

    def send_joint_angles(self, rad1, rad2, rad3, rad4, rad5, rad6, mode=1):
        """严格大端序打包发送函数"""
        if not self.serial_port or not self.serial_port.is_open:
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
            self.serial_port.write(data)
            self.serial_port.flush()  
        except Exception as e:
            self.get_logger().error(f"❌ 串口数据发射异常: {e}")

    def target_callback(self, msg):
        # 🛡️ 已经完成抓取，忽略后续所有坐标
        if self.has_executed:
            return

        cam_x = msg.point.x  
        cam_z = msg.point.z  

        # 🎯 收集坐标样本
        self.collected_x.append(cam_x)
        self.collected_y.append(0.0)      # 原逻辑只用 cam_x 和 cam_z，y 为占位
        self.collected_z.append(cam_z)

        # ⏳ 样本数不足，继续等待
        if len(self.collected_x) < self.samples_needed:
            self.get_logger().info(f"📊 已收集 {len(self.collected_x)}/{self.samples_needed} 帧坐标...")
            return

        # ✅ 样本充足，计算平均值
        avg_x = sum(self.collected_x) / len(self.collected_x)
        avg_z = sum(self.collected_z) / len(self.collected_z)
        self.get_logger().info(f"🎯 坐标平均完成: x={avg_x:.3f}, z={avg_z:.3f} （共 {len(self.collected_x)} 帧）")

        # 用平均值覆盖 cam_x、cam_z，走后续逻辑
        cam_x = avg_x
        cam_z = avg_z

        # ==========================================
        # 📛 标记已执行，此后消息一律忽略
        # ==========================================
        self.has_executed = True

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
        self.get_logger().info(f"📊 最终下发底层真实弧度 -> J1:{j1:.3f} | J2:{j2:.3f} | J3:{j3:.3f} | J4:{j4:.3f}")

        self.get_logger().info("🤖 动作 1: 手臂高高拱起，向目标上方平滑逼近...")
        # 💡 优化：把高举弧度从 +0.15 缩减为 +0.06，减少俯冲惯性和晃动
        self.send_joint_angles(j1, j2 + 0.06, j3, j4 - 0.06, j5, 1.57, mode=2) 
        time.sleep(2.0)
        
        self.get_logger().info("🤖 动作 2: 向下沉降，预留充足时间消除重力震动...")
        self.send_joint_angles(j1, j2, j3, j4, j5, 1.57, mode=2) 
        # 💡 强力优化：从 1.0 秒暴增到 2.5 秒！给大臂完全沉降并平息余震的时间，确保高度100%到位
        time.sleep(3.0) 
        
        self.get_logger().info("🤖 动作 3: 全身完全静止，合拢夹爪进行完美咬合！")
        self.send_joint_angles(j1, j2, j3, j4, j5, -0.4, mode=2) 
        time.sleep(1.2)
        
        self.get_logger().info("🤖 动作 4: 成功捕获，垂直向上抬起...")
        self.send_joint_angles(j1, j2 + 0.3, j3, j4 - 0.3, j5, -0.4, mode=2)
        time.sleep(2.0)
        
        self.get_logger().info("🏠 动作 5: 安全归位（直立全零）。")
        self.send_joint_angles(0.0, 0.0, 0.0, 0.0, 0.0, -0.4, mode=2)
        time.sleep(1.0) 

def main(args=None):
    rclpy.init(args=args)
    node = ArmGrabberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()