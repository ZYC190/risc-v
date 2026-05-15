import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

# ==========================================
# 分布式节点：ESP32 物联协议转换器 (通讯兵)
# ==========================================
class Esp32MqttBridge(Node):
    def __init__(self):
        super().__init__('esp32_mqtt_bridge_node')
        
        # 1. 配置 MQTT 枪机
        self.mqtt_broker = "127.0.0.1"
        self.mqtt_port = 1883
        self.mqtt_topic = "edge/light/cmd"
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "ROS2_Bridge")
        
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("✅ [通讯兵] MQTT 连接成功，武器已上膛！")
        except Exception as e:
            self.get_logger().error(f"❌ [通讯兵] MQTT 连接失败: {e}")

        # 2. 配置 ROS2 监听器 (监听 UI 控制台发来的指令)
        self.subscription = self.create_subscription(
            String,
            '/esp32_cmd',  # 必须和 UI 界面发布的 Topic 名字完全一致
            self.ros_to_mqtt_callback,
            10
        )
        self.get_logger().info("📡 [通讯兵] 正在监听 /esp32_cmd 指令频道...")

    def ros_to_mqtt_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"📥 收到控制台指令: {cmd}")
        
        # 将 ROS2 里的字符串，翻译成物理 MQTT 指令打出去！
        try:
            self.mqtt_client.publish(self.mqtt_topic, cmd)
            self.get_logger().info(f"💡 已通过 MQTT 发射物理指令: {cmd}")
        except Exception as e:
            self.get_logger().error(f"发射失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Esp32MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()