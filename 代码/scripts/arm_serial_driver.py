#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class WheeltecArmDriver(Node):
    def __init__(self):
        super().__init__('wheeltec_table_arm')
        
        # 1. 声明与获取参数
        self.declare_parameter('usart_port_name', '/dev/wheeltec_arm')
        self.declare_parameter('serial_baud_rate', 115200)
        
        port_name = self.get_parameter('usart_port_name').value
        baud_rate = self.get_parameter('serial_baud_rate').value
        
        # 2. 连接 STM32 串口
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=0.1)
            self.get_logger().info(f"🔥 成功连接 STM32 串口: {port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口连接失败，请检查连线或权限: {e}")
            return
            
        # 给单片机串口芯片 1.0 秒的稳定电平时间，防止开机数据丢包
        self.get_logger().info("等待串口硬件接收器整备...")
        time.sleep(1.0)
            
        # 3. 订阅 ROS 2 核心控制话题
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10)
        self.teleop_sub = self.create_subscription(
            JointState, 'arm_teleop', self.arm_teleop_callback, 10)
            
        # 4. 发送开机回正指令 (严格遵循 Moveit 16字节控制帧)
        # 初始目标弧度：关节4设为 -1.57 (即前倾低矮姿态)
        self.get_logger().info("正在向单片机发射初始化归位密令...")
        self.send_to_stm32([0.0, +1, -1.57,-1.57,0.0, 0.0], mode=2)

    def send_to_stm32(self, angles, mode):
        """核心打包与发送函数 - 严格对照官方 Moveit 16字节通讯协议"""
        tx = bytearray(16)
        tx[0] = 0xAA  # [0]: 帧头 固定 0xAA
        
        # 【核心修正 1】针对蓝图中提示的“关节3/4固件取反”特性
        # 如果实测发现这两个关节运动方向与 ROS 实际方向相反，可将对应位置的 1 改为 -1
        direction_mask = [1, 1, 1, 1, 1, 1] 
        
        # 转换 6 个关节的弧度值
        for i in range(6):
            # 【核心修正 2】严格执行官方公式：angle_rad * 1000
            target_rad = angles[i] * direction_mask[i]
            val = int(target_rad * 1000)
            
            # Python 处理负数转 16 位有符号整型 (short)
            val = val & 0xFFFF
            
            # 【核心修正 3】严格遵循蓝图：大端序（高字节在前，低字节在后）
            tx[1 + i*2] = (val >> 8) & 0xFF  # 高 8 位 (e.g. [1], [3], [5]...)
            tx[2 + i*2] = val & 0xFF         # 低 8 位 (e.g. [2], [4], [6]...)
            
        tx[13] = mode  # [13]: 控制模式 (2=跟随模式柔和PID，1=默认强力PID)
        
        # 【核心修正 4】严格执行蓝图校验和：从 [0] 到 [13] 连续异或 (XOR)
        check_sum = 0
        for i in range(14):
            check_sum ^= tx[i]
        tx[14] = check_sum  # [14]: 校验和
        
        tx[15] = 0xBB  # [15]: 帧尾 固定 0xBB
        
        # 通过串口死线发射
        try:
            self.serial_port.write(tx)
        except Exception as e:
            self.get_logger().error(f"数据发射失败: {e}")

    def joint_states_callback(self, msg):
        """接收到运动规划控制指令"""
        if len(msg.position) >= 6:
            self.send_to_stm32(msg.position[:6], mode=1)

    def arm_teleop_callback(self, msg):
        """接收到遥控/手柄控制指令"""
        if len(msg.position) >= 6:
            self.send_to_stm32(msg.position[:6], mode=1)

def main(args=None):
    rclpy.init(args=args)
    node = WheeltecArmDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()