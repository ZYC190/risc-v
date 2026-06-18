#!/usr/bin/env python3
"""
==========================================
 MQTT → ROS 2 桥接节点 (战车遥控兵)
==========================================
功能:
  1. 连接 MQTT Broker (10.46.106.188:1883)
  2. 订阅 phone/cmd_vel 话题，接收手机摇杆 JSON
  3. 解析 JSON → geometry_msgs/Twist 发布到 /cmd_vel
  4. 安全机制：超过 0.5 秒未收到消息 → 自动刹车归零

数据流:
  手机摇杆 → MQTT phone/cmd_vel (JSON)
  → 本节点解析 → ROS 2 /cmd_vel (Twist)
  → 底盘驱动 → 战车出动！

运行方式:
  python3 mqtt_to_ros2_bridge.py
"""

import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt


class MqttToRos2Bridge(Node):
    """MQTT → ROS 2 /cmd_vel 桥接节点"""

    def __init__(self):
        super().__init__('mqtt_to_ros2_bridge')

        # ==================== 配置参数 ====================
        # MQTT Broker 地址（小车端 Mosquitto）
        self.mqtt_broker = "127.0.0.1"
        self.mqtt_port = 1883
        self.mqtt_topic = "phone/cmd_vel"
        self.mqtt_voice_topic = "phone/voice_text"  # 🎤 语音识别文字

        # 超时断连刹车时间（秒）
        self.timeout = 0.5

        # 最后收到消息的时间戳
        self.last_msg_time = time.time()

        # ==================== ROS 2 Publisher ====================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_logger().info("📡 ROS 2 Publisher: /cmd_vel 已就绪")

        # ==================== MQTT 客户端 ====================
        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            "ROS2_CmdVel_Bridge"
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 连接失败: {e}")
            return

        # ==================== 超时检测定时器 ====================
        # 每 0.1 秒检查一次，超过 timeout 秒未收到消息则刹车
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info("✅ MQTT → ROS 2 桥接节点启动完成！")
        self.get_logger().info(f"   监听 MQTT: {self.mqtt_topic}")
        self.get_logger().info(f"   超时刹车: {self.timeout}s")

    # ==================== MQTT 事件回调 ====================

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT 连接成功回调"""
        if reason_code == 0:
            self.get_logger().info(
                f"✅ 已连接 MQTT Broker: {self.mqtt_broker}:{self.mqtt_port}"
            )
            client.subscribe(self.mqtt_topic)
            client.subscribe(self.mqtt_voice_topic)  # 🎤 语音话题
            self.get_logger().info(f"✅ 已订阅 MQTT 话题: {self.mqtt_topic}")
            self.get_logger().info(f"✅ 已订阅语音话题: {self.mqtt_voice_topic}")
        else:
            self.get_logger().error(
                f"❌ MQTT 连接失败，返回码: {reason_code}"
            )

    def _smart_decode(self, raw_bytes):
        """多编码尝试解码，优先中文编码，返回 (text, encoding_name)"""
        # 打印原始 hex 方便调试
        hex_preview = raw_bytes[:80].hex() if len(raw_bytes) > 80 else raw_bytes.hex()
        self.get_logger().warn(f"🔍 原始 HEX ({len(raw_bytes)}B): {hex_preview}")

        # 优先尝试中文编码（gbk/gb2312/gb18030 系列），再尝试 utf-8
        for enc in ['gb18030', 'gbk', 'gb2312', 'utf-8']:
            try:
                text = raw_bytes.decode(enc)
                # 避免 latin-1 陷阱：必须不含替换字符
                if '\ufffd' not in text:
                    # 如果解码结果包含中文，基本确定是正确的
                    if any('\u4e00' <= c <= '\u9fff' for c in text):
                        self.get_logger().warn(f"✅ 中文编码匹配: {enc} → 「{text[:60]}」")
                        return text, enc
                    # 纯 ASCII/英文也接受（如 "hello"）
                    if text.isascii():
                        self.get_logger().warn(f"✅ ASCII 编码匹配: {enc} → 「{text[:60]}」")
                        return text, enc
                    # 含非 ASCII 但非中文 → 编码可能不对，继续尝试
            except (UnicodeDecodeError, UnicodeError):
                continue

        # 全部失败，用 utf-8 replace 兜底
        fallback = raw_bytes.decode('utf-8', errors='replace')
        self.get_logger().warn(f'⚠️ 所有编码尝试失败，使用 utf-8 replace 兜底 → 「{fallback[:60]}」')
        return fallback, 'utf-8(replace)'

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT 收到消息回调"""
        # 多编码智能解码
        payload_str, enc = self._smart_decode(msg.payload)

        # 🎤 语音文字消息
        if msg.topic == self.mqtt_voice_topic:
            # 先尝试当 JSON 解析
            text = ""
            try:
                data = json.loads(payload_str)
                text = data.get("text", "")
            except (json.JSONDecodeError, ValueError, TypeError):
                # 不是 JSON，直接当纯文本使用
                text = payload_str.strip()

            if text:
                self.get_logger().info(f"🎤 语音识别文字: 「{text}」 (编码:{enc})")

                # 🚀 转发到 robot/voice_cmd 让 agv_master_node.py 触发导航
                try:
                    forward_payload = json.dumps({"text": text}, ensure_ascii=False)
                    self.mqtt_client.publish("robot/voice_cmd", forward_payload)
                    self.get_logger().info(
                        f"📤 已转发到 MQTT robot/voice_cmd → agv_master_node 将处理导航"
                    )
                except Exception as e:
                    self.get_logger().warning(f"⚠️ 转发失败: {e}")
            else:
                self.get_logger().warning(f"⚠️ 语音消息为空 ({enc}): {payload_str[:100]}")
            return

        # 🕹️ 速度指令消息
        try:
            data = json.loads(payload_str)
            linear_x = float(data.get("linear_x", 0.0))
            angular_z = float(data.get("angular_z", 0.0))
        except (json.JSONDecodeError, ValueError, TypeError) as e:
            self.get_logger().warning(f"⚠️ 无效 JSON: {payload_str} | 错误: {e}")
            return

        # 更新最后收到消息的时间
        self.last_msg_time = time.time()

        # 发布 Twist 到 /cmd_vel
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

        # 只在非零速度时打印日志（减少控制台刷屏）
        if linear_x != 0.0 or angular_z != 0.0:
            self.get_logger().info(
                f"🏎️  MQTT → /cmd_vel | linear_x: {linear_x:.3f}, angular_z: {angular_z:.3f}"
            )

    # ==================== 安全机制 ====================

    def _watchdog_callback(self):
        """看门狗定时器：超时自动刹车"""
        elapsed = time.time() - self.last_msg_time
        if elapsed > self.timeout:
            # 超过超时时间，发布零速指令刹车
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            # 避免重复打印刹车日志
            if not getattr(self, '_brake_logged', False):
                self.get_logger().warn(
                    f"🛑 超时刹车！{self.timeout}s 未收到 MQTT 消息，/cmd_vel 已归零"
                )
                self._brake_logged = True
            return

        # 恢复消息时重置刹车日志标记
        if getattr(self, '_brake_logged', False):
            self._brake_logged = False
            self.get_logger().info("🟢 MQTT 信号恢复，遥控已接管")

    # ==================== 清理 ====================

    def destroy_node(self):
        """节点销毁前清理资源"""
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.get_logger().info("🔌 MQTT 连接已断开")
        super().destroy_node()


# ==================== 主函数 ====================

def main(args=None):
    rclpy.init(args=args)
    node = MqttToRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 收到停机指令，正在安全关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()