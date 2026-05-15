#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
from geometry_msgs.msg import Point 

class ArmGrabberNode(Node):
    def __init__(self):
        super().__init__('arm_grabber_node')
        
        # ==========================================
        # 1. 串口初始化
        # ==========================================
        self.port_name = '/dev/wheeltec_controller' # 串口名称
        self.baud_rate = 115200
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=0.5)
            self.get_logger().info(f"✅ 成功连接底层 STM32: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            self.serial_port = None

        # 订阅目标坐标
        self.subscription = self.create_subscription(
            Point,
            '/target_coordinate',
            self.target_callback,
            10)

        # ==========================================
        # 2. 官方机械臂结构参数
        # ==========================================
        self.link_a = 0.105         # 机械参数 a
        self.link_b = 0.105         # 机械参数 b
        self.link_c = 0.145         # 机械参数 c
        self.link_h = 0.080         # 机械参数 h
        
        self.x_offset = 0.0         # X轴抓取微调偏差 (米)
        self.y_offset = 0.0         # Y轴抓取微调偏差 (米)
        self.auxiliary_angle = 0.0  # 辅助角 (弧度)
        
        # 计算夹爪可触底的基础角度
        val = (self.link_c - self.link_h) / self.link_a
        val = max(-1.0, min(1.0, val)) # 防止浮点数溢出
        self.basic_angle = math.acos(val) 
        self.get_logger().info(f"🔧 初始化完成，基础角为: {self.basic_angle:.4f} rad")

    def send_joint_angles(self, rad1, rad2, rad3, rad4, rad5, rad6, mode=1):
        """将 6 个弧度转换为 16 字节协议并发送给 STM32"""
        if not self.serial_port or not self.serial_port.is_open:
            return

        # 转换为 short (*1000)
        j1 = int(rad1 * 1000)
        j2 = int(rad2 * 1000)
        j3 = int(rad3 * 1000)
        j4 = int(rad4 * 1000)
        j5 = int(rad5 * 1000)
        j6 = int(rad6 * 1000)

        # 组装数据帧
        data = bytearray(16)
        data[0] = 0xAA  
        
        struct.pack_into('>h', data, 1, j1)
        struct.pack_into('>h', data, 3, j2)
        struct.pack_into('>h', data, 5, j3)
        struct.pack_into('>h', data, 7, j4)
        struct.pack_into('>h', data, 9, j5)
        struct.pack_into('>h', data, 11, j6)
        
        data[13] = mode 

        # 计算异或校验和
        check_sum = 0
        for i in range(14):
            check_sum ^= data[i]
        data[14] = check_sum
        
        data[15] = 0xBB 

        self.serial_port.write(data)

    def inverse_kinematics(self, x, y, rotate=0.0):
        """
        官方逆运动学逻辑：将二维平面坐标转换为机械臂整体姿态
        """
        true_x = x + self.x_offset
        true_y = y + self.y_offset

        # 1. 计算云台目标运动角度 (pedestal_angle)
        if true_y == 0: 
            true_y = 0.0001 # 防除零报错
            
        pedestal_angle = math.degrees(math.atan(abs(true_x / true_y)))
        if true_x > 0:
            pedestal_angle = pedestal_angle
        else:
            pedestal_angle = -pedestal_angle
        
        pedestal_angle_rad = math.radians(pedestal_angle)

        # 2. 计算机械臂夹角 (arm_angle) - 已修复 math 拼写错误
        caculate_A = self.link_a * math.sin(self.basic_angle) + math.sin(self.auxiliary_angle) * self.link_c
        caculate_B = self.link_a * math.cos(self.basic_angle) + math.cos(self.auxiliary_angle) * self.link_c
        
        caculate_C = math.sqrt(math.pow(true_x, 2) + math.pow(true_y, 2)) - self.link_b
        
        val_for_D = caculate_C / math.sqrt(math.pow(caculate_A, 2) + math.pow(caculate_B, 2))
        val_for_D = max(-1.0, min(1.0, val_for_D))
        caculate_D = math.acos(val_for_D)
        
        caculate_E = math.atan2(caculate_B, caculate_A)
        caculate_G = caculate_E - caculate_D
        
        arm_angle_rad = caculate_G

        # 3. 计算夹取旋转角度 (hand_angle)
        hand_angle = rotate + 90
        if hand_angle > 45:
            hand_angle = hand_angle - 90
        hand_angle_rad = math.radians(hand_angle)

        return pedestal_angle_rad, arm_angle_rad, hand_angle_rad

    def target_callback(self, msg):
        """当收到目标坐标时，触发抓取序列"""
        x = msg.x
        y = msg.y
        
        self.get_logger().info(f"🎯 视觉雷达锁定目标: X={x:.3f}, Y={y:.3f}")
        
        # 获取 3 个核心控制角
        pedestal, arm, hand = self.inverse_kinematics(x, y, rotate=0.0)
        
        # ==========================================
        # ⚠️ 姿态微调区：控制下探深度
        # ==========================================
        # 增大此值让机械臂更往下压，减小此值防止砸桌面
        down_offset = -0.9
        
        j1 = pedestal                       # 云台转动
        j2 = arm + down_offset              # 大臂下压
        j3 = -arm - (down_offset * 0.5)     # 小臂配合弯曲
        j4 = -1.57                          # 腕部维持垂直向下 (-90度)
        j5 = hand                           # 夹爪旋转
        
        # --- 实战状态机 ---
        
        self.get_logger().info("🤖 动作 1: 移动到正上方预抓取位置...")
        # 让大臂抬高 0.3，不要马上压下去
        self.send_joint_angles(j1, j2 + 0.3, j3, j4, j5, 1.57, mode=2) 
        time.sleep(2.0)
        
        self.get_logger().info("🤖 动作 2: 直线下探！")
        # 压到目标高度
        self.send_joint_angles(j1, j2, j3, j4, j5, 1.57, mode=2) 
        time.sleep(1.0)
        
        self.get_logger().info("🤖 动作 3: 闭合夹爪！")
        # 将 0.0 改成负数，比如 -0.3 甚至 -0.5，让它往里使劲捏！
        self.send_joint_angles(j1, j2, j3, j4, j5, -0.3, mode=1) 
        time.sleep(1.0)
        
        self.get_logger().info("🤖 动作 4: 抓取成功，抬起复位待命！")
        # 抬起的时候，最后一个参数也要改成一样的负数，保持捏紧状态！
        self.send_joint_angles(0.0, 0.5, -0.5, -1.57, 0.0, -0.3, mode=2)


def main(args=None):
    rclpy.init(args=args)
    node = ArmGrabberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()