#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

class WheeltecArmDriver(Node):
    def __init__(self):
        super().__init__('wheeltec_table_arm')
        
        # 1. 声明参数（从之前的 launch 文件移植过来）
        self.declare_parameter('usart_port_name', '/dev/wheeltec_controller')
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
            
        # 3. 订阅 ROS 2 的标准关节控制话题
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10)
        self.teleop_sub = self.create_subscription(
            JointState, 'arm_teleop', self.arm_teleop_callback, 10)
            
        # 4. 发送开机回正指令 (对应 C++ 里的 init_joint_states)
        self.get_logger().info("正在发送机械臂初始化回正指令...")
        self.send_to_stm32([0.0, 0.0, 0.0, -1.57, 0.0, 0.0], mode=2)

    def send_to_stm32(self, angles, mode):
        """核心打包与发送函数"""
        tx = bytearray(16)
        tx[0] = 0xAA  # 帧头 FRAME_HEADER_ARM
        
        # 转换 6 个舵机角度
        for i in range(6):
            # 将角度放大1000倍
            val = int(angles[i] * 1000)
            # Python 处理负数转16位整型的极客写法 (类似 C++ 的 short)
            val = val & 0xFFFF
            tx[1 + i*2] = (val >> 8) & 0xFF  # 高 8 位
            tx[2 + i*2] = val & 0xFF         # 低 8 位
            
        tx[13] = mode  # 控制模式
        
        # 计算 Check_Sum (前 14 个字节按位异或)
        check_sum = 0
        for i in range(14):
            check_sum ^= tx[i]
        tx[14] = check_sum
        
        tx[15] = 0xBB  # 帧尾 FRAME_TAIL_ARM
        
        # 通过串口发射！
        try:
            self.serial_port.write(tx)
        except Exception as e:
            self.get_logger().error(f"发送数据失败: {e}")

    def joint_states_callback(self, msg):
        """接收到正常控制指令"""
        if len(msg.position) >= 6:
            self.send_to_stm32(msg.position[:6], mode=1)

    def arm_teleop_callback(self, msg):
        """接收到遥控指令"""
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