import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Esp32MqttBridge(Node):
    """ROS2 <-> MQTT bridge for ESP32 smart-home demo."""

    def __init__(self):
        super().__init__("esp32_mqtt_bridge_node")

        self.declare_parameter("mqtt_broker", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("cmd_topic", "edge/light/cmd")
        self.declare_parameter("status_topic", "edge/esp32/status")

        self.mqtt_broker = self.get_parameter("mqtt_broker").value
        self.mqtt_port = int(self.get_parameter("mqtt_port").value)
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.esp32_status_pub = self.create_publisher(String, "/esp32_status", 10)
        self.subscription = self.create_subscription(
            String,
            "/esp32_cmd",
            self.ros_to_mqtt_callback,
            10,
        )

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "ROS2_ESP32_Bridge")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        self.connect_mqtt()

        self.get_logger().info("ESP32 bridge ready")
        self.get_logger().info(f"ROS  -> MQTT: /esp32_cmd  -> {self.cmd_topic}")
        self.get_logger().info(f"MQTT -> ROS : {self.status_topic} -> /esp32_status")

    def connect_mqtt(self):
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"MQTT connected: {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as exc:
            self.get_logger().error(f"MQTT connect failed: {exc}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.status_topic)
            self.get_logger().info(f"Subscribed MQTT status topic: {self.status_topic}")
        else:
            self.get_logger().error(f"MQTT connect rc={rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="replace")
        ros_msg = String()

        # Keep UI display short and readable, but preserve useful state.
        try:
            data = json.loads(payload)
            event = data.get("event", "STATUS")
            light = "ON" if data.get("light") else "OFF"
            fan = "ON" if data.get("fan") else "OFF"
            alarm = "ON" if data.get("alarm") else "OFF"
            ip = data.get("ip", "")
            ros_msg.data = f"{event} | light={light} fan={fan} alarm={alarm} {ip}".strip()
        except Exception:
            ros_msg.data = payload

        self.esp32_status_pub.publish(ros_msg)
        self.get_logger().info(f"MQTT status -> /esp32_status: {ros_msg.data}")

    def ros_to_mqtt_callback(self, msg):
        cmd = msg.data.strip().upper()
        if not cmd:
            return

        try:
            result = self.mqtt_client.publish(self.cmd_topic, cmd, qos=0, retain=False)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f"/esp32_cmd -> MQTT {self.cmd_topic}: {cmd}")
            else:
                self.get_logger().error(f"MQTT publish failed rc={result.rc}: {cmd}")
        except Exception as exc:
            self.get_logger().error(f"MQTT publish exception: {exc}")

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
