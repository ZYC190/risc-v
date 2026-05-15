#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import json
import threading

# ================= 战前配置 =================
SERIAL_PORT = '/dev/air_sensor' 
BAUD_RATE = 9600
# ============================================

class AirSensorNode(Node):
    def __init__(self):
        super().__init__('air_sensor_node')
        # 建立广播通道，发送给 UI 和 小爱大脑
        self.publisher_ = self.create_publisher(String, '/air_sensor_data', 10)
        self.get_logger().info(f"🚀 环境雷达节点已启动，正在连接 {SERIAL_PORT}...")
        
        # 开启独立线程去读串口，防止阻塞 ROS2 主干
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def decode_airmod(self, data):
        """解码核心算法"""
        if len(data) != 17: return None
        checksum = sum(data[0:16]) & 0xFF
        if data[0] == 0x3C and data[1] == 0x02 and checksum == data[16]:
            co2  = (data[2] << 8) | data[3]
            jq   = (data[4] << 8) | data[5]
            voc  = (data[6] << 8) | data[7]
            pm25 = (data[8] << 8) | data[9]
            pm10 = (data[10] << 8) | data[11]

            temp_minus = 1 if (data[12] & 0x80) else 0
            temp_float = (data[12] & 0x7F) + (data[13] * 0.1)
            if temp_minus: temp_float = -temp_float

            humi_float = data[14] + (data[15] * 0.1)

            return {
                "CO2": co2, "甲醛": jq, "VOC": voc,
                "PM2.5": pm25, "PM10": pm10,
                "温度": round(temp_float, 1), "湿度": round(humi_float, 1)
            }
        return None

    def read_serial_loop(self):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
            self.get_logger().info("✅ 串口连接成功！开始广播数据...")
            
            while rclpy.ok():
                if ser.read(1) == b'\x3C' and ser.read(1) == b'\x02':
                    rest_of_data = ser.read(15)
                    if len(rest_of_data) == 15:
                        full_frame = b'\x3C\x02' + rest_of_data
                        result_dict = self.decode_airmod(full_frame)
                        
                        if result_dict:
                            # 将字典转为 JSON 字符串广播出去
                            msg = String()
                            msg.data = json.dumps(result_dict)
                            self.publisher_.publish(msg)
                            
                time.sleep(0.5) # 控制发送频率为 2Hz
        except Exception as e:
            self.get_logger().error(f"❌ 串口读取异常: {e}。请检查是否执行了 sudo chmod 777 {SERIAL_PORT}")

def main(args=None):
    rclpy.init(args=args)
    node = AirSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()