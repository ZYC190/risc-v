#!/usr/bin/env python3
"""Unified MQTT <-> ROS2 bridge for the home service robot."""

import json
import math
import os
import time

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


ARM_OFFSETS_FILE = os.environ.get(
    "ROBOT_ARM_OFFSETS_FILE",
    "/home/zyc/robot2/config/arm_grab_offsets.json",
)


ARM_PLAIN_COMMANDS = {
    "GRAB_BOTTLE": "GRAB_BOTTLE",
    "GRAB": "GRAB",
    "SCAN": "SCAN",
    "RESET": "RESET",
    "STOP": "STOP",
    "RELEASE": "RELEASE",
    "OPEN": "RELEASE",
    "OPEN_GRIPPER": "RELEASE",
    "CLOSE": "CLOSE",
    "CLOSE_GRIPPER": "CLOSE",
}


class UnifiedMqttBridge(Node):
    def __init__(self):
        super().__init__("unified_mqtt_bridge")

        self.declare_parameter("mqtt_broker", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("cmd_vel_topic", "phone/cmd_vel")
        self.declare_parameter("voice_topic", "phone/voice_text")
        self.declare_parameter("voice_forward_topic", "robot/voice_cmd")
        self.declare_parameter("arm_topic", "phone/arm_cmd")
        self.declare_parameter("arm_status_topic", "home/arm/status")
        self.declare_parameter("esp32_cmd_topic", "edge/light/cmd")
        self.declare_parameter("esp32_status_topic", "edge/esp32/status")
        self.declare_parameter("patrol_cmd_topic", "home/patrol/cmd")
        self.declare_parameter("patrol_status_topic", "home/patrol/status")
        self.declare_parameter("navigation_goal_topic", "home/navigation/goal")
        self.declare_parameter(
            "navigation_system_cmd_topic", "home/navigation/system_cmd"
        )
        self.declare_parameter(
            "navigation_system_status_topic", "home/navigation/system_status"
        )
        self.declare_parameter("security_alert_topic", "home/security/alert")
        self.declare_parameter("cmd_vel_timeout", 0.5)

        self.broker = str(self.get_parameter("mqtt_broker").value)
        self.port = int(self.get_parameter("mqtt_port").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.voice_topic = str(self.get_parameter("voice_topic").value)
        self.voice_forward_topic = str(self.get_parameter("voice_forward_topic").value)
        self.arm_topic = str(self.get_parameter("arm_topic").value)
        self.arm_status_topic = str(
            self.get_parameter("arm_status_topic").value
        )
        self.esp32_cmd_topic = str(self.get_parameter("esp32_cmd_topic").value)
        self.esp32_status_topic = str(self.get_parameter("esp32_status_topic").value)
        self.patrol_cmd_topic = str(self.get_parameter("patrol_cmd_topic").value)
        self.patrol_status_topic = str(
            self.get_parameter("patrol_status_topic").value
        )
        self.navigation_goal_topic = str(
            self.get_parameter("navigation_goal_topic").value
        )
        self.navigation_system_cmd_topic = str(
            self.get_parameter("navigation_system_cmd_topic").value
        )
        self.navigation_system_status_topic = str(
            self.get_parameter("navigation_system_status_topic").value
        )
        self.security_alert_topic = str(
            self.get_parameter("security_alert_topic").value
        )
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)

        # 非导航模式直接控制底盘；巡查模式必须进入 Nav2 的平滑与防撞链。
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.arm_teleop_pub = self.create_publisher(JointState, "arm_teleop", 10)
        self.arm_cmd_pub = self.create_publisher(String, "/arm_cmd", 10)
        self.esp32_status_pub = self.create_publisher(String, "/esp32_status", 10)
        self.patrol_cmd_pub = self.create_publisher(String, "/home/patrol/cmd", 10)
        self.navigation_system_cmd_pub = self.create_publisher(
            String, "/home/navigation/system_cmd", 10
        )
        self.esp32_cmd_sub = self.create_subscription(
            String, "/esp32_cmd", self._on_esp32_ros_command, 10
        )
        self.patrol_status_sub = self.create_subscription(
            String, "/home/patrol/status", self._on_patrol_ros_status, 10
        )
        self.security_alert_sub = self.create_subscription(
            String, "/home/security/alert", self._on_security_ros_alert, 10
        )
        self.navigation_system_status_sub = self.create_subscription(
            String,
            "/home/navigation/system_status",
            self._on_navigation_system_status,
            10,
        )
        self.arm_status_sub = self.create_subscription(
            String, "/arm_status", self._on_arm_ros_status, 10
        )

        self.last_cmd_vel_time = 0.0
        self.brake_sent = True
        self.navigation_mode = False
        self.arm_offsets = {
            "x_offset": -0.07,
            "y_offset": -0.05,
            "z_offset": 0.05,
        }
        self._load_arm_offsets()
        self.mqtt_connected = False
        self.shutting_down = False

        self.mqtt_client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="ROS2_Unified_Control_Bridge",
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.mqtt_client.connect_async(self.broker, self.port, keepalive=60)
        self.mqtt_client.loop_start()

        self.watchdog_timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info("统一 MQTT-ROS2 桥接节点已启动")
        self.get_logger().info(f"MQTT Broker: {self.broker}:{self.port}")
        self.get_logger().info("已启用：底盘、语音、机械臂、ESP32 控制与状态回传")

    @staticmethod
    def _valid_arm_offsets(offsets):
        return all(
            math.isfinite(value) and -0.5 <= value <= 0.5
            for value in offsets.values()
        )

    def _load_arm_offsets(self):
        try:
            with open(ARM_OFFSETS_FILE, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            offsets = {
                key: float(data[key])
                for key in ("x_offset", "y_offset", "z_offset")
            }
            if not self._valid_arm_offsets(offsets):
                raise ValueError("偏移超出 -0.5 到 0.5 米范围")
            self.arm_offsets = offsets
            self.get_logger().info(
                f"已加载持久化机械臂抓取参数: {ARM_OFFSETS_FILE}"
            )
        except FileNotFoundError:
            self.get_logger().info("尚无持久化机械臂抓取参数，使用程序默认值")
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"读取持久化机械臂抓取参数失败，使用程序默认值: {exc}"
            )

    def _save_arm_offsets(self):
        temp_file = f"{ARM_OFFSETS_FILE}.tmp.{os.getpid()}"
        try:
            os.makedirs(os.path.dirname(ARM_OFFSETS_FILE), exist_ok=True)
            with open(temp_file, "w", encoding="utf-8") as stream:
                json.dump(self.arm_offsets, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_file, ARM_OFFSETS_FILE)
            self.get_logger().info("机械臂抓取参数已持久化保存")
            return True
        except OSError as exc:
            self.get_logger().error(f"持久化机械臂抓取参数失败: {exc}")
            try:
                os.unlink(temp_file)
            except OSError:
                pass
            return False

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.mqtt_connected = False
            self.get_logger().error(f"MQTT 连接失败，返回码：{reason_code}")
            return

        self.mqtt_connected = True
        topics = (
            self.cmd_vel_topic,
            self.voice_topic,
            self.arm_topic,
            self.esp32_status_topic,
            self.patrol_cmd_topic,
            self.navigation_goal_topic,
            self.navigation_system_cmd_topic,
        )
        for topic in topics:
            client.subscribe(topic)
        self.get_logger().info("MQTT 已连接，7 个控制/状态主题订阅成功")
        self._publish_arm_offsets_status()

    def _on_mqtt_disconnect(
        self, client, userdata, disconnect_flags, reason_code, properties
    ):
        self.mqtt_connected = False
        if self.shutting_down:
            return
        self.get_logger().warning(f"MQTT 连接断开，正在自动重连：{reason_code}")

    @staticmethod
    def _decode(payload):
        for encoding in ("utf-8", "gb18030", "gbk"):
            try:
                return payload.decode(encoding), encoding
            except UnicodeDecodeError:
                pass
        return payload.decode("utf-8", errors="replace"), "utf-8(replace)"

    def _on_mqtt_message(self, client, userdata, message):
        payload, encoding = self._decode(message.payload)
        try:
            if message.topic == self.cmd_vel_topic:
                self._handle_cmd_vel(payload)
            elif message.topic == self.voice_topic:
                self._handle_voice(payload, encoding)
            elif message.topic == self.arm_topic:
                self._handle_arm(payload)
            elif message.topic == self.esp32_status_topic:
                self._handle_esp32_status(payload)
            elif message.topic == self.patrol_cmd_topic:
                self._handle_patrol_command(payload)
            elif message.topic == self.navigation_goal_topic:
                self._handle_navigation_goal(payload)
            elif message.topic == self.navigation_system_cmd_topic:
                self._handle_navigation_system_command(payload)
        except Exception as exc:
            self.get_logger().error(
                f"处理 MQTT 消息失败，主题={message.topic}：{exc}"
            )

    def _handle_cmd_vel(self, payload):
        try:
            data = json.loads(payload)
            linear_x = float(data.get("linear_x", 0.0))
            linear_y = float(data.get("linear_y", 0.0))
            angular_z = float(data.get("angular_z", 0.0))
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"底盘指令格式错误：{exc}")
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._publish_drive_twist(twist)
        self.last_cmd_vel_time = time.monotonic()
        self.brake_sent = False

    def _publish_drive_twist(self, twist):
        publisher = self.nav_cmd_vel_pub if self.navigation_mode else self.cmd_vel_pub
        publisher.publish(twist)

    def _handle_voice(self, payload, encoding):
        try:
            data = json.loads(payload)
            text = str(data.get("text", "")).strip()
        except (json.JSONDecodeError, TypeError, ValueError):
            text = payload.strip()

        if not text:
            self.get_logger().warning("收到空的手机语音文字，已忽略")
            return

        forwarded = json.dumps({"text": text}, ensure_ascii=False)
        result = self.mqtt_client.publish(self.voice_forward_topic, forwarded)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(f"家长文字已转发给机器人播报：{text}")
        else:
            self.get_logger().error(f"语音文字转发失败，返回码：{result.rc}")

    def _publish_arm_command(self, command):
        msg = String()
        msg.data = command
        self.arm_cmd_pub.publish(msg)
        self.get_logger().info(f"机械臂动作指令：{command}")

    def _publish_arm_offsets(self):
        self._publish_arm_command(
            json.dumps(
                {"type": "set_offsets", **self.arm_offsets},
                ensure_ascii=False,
            )
        )

    def _publish_arm_offsets_status(self):
        if not self.mqtt_connected:
            return
        payload = json.dumps(
            {
                "event": "arm_offsets",
                **self.arm_offsets,
                "message": "机械臂抓取参数已同步",
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            },
            ensure_ascii=False,
        )
        self.mqtt_client.publish(
            self.arm_status_topic, payload, qos=0, retain=True
        )

    def _handle_arm(self, payload):
        plain_command = payload.strip().upper()
        if plain_command in ARM_PLAIN_COMMANDS:
            if plain_command in {"GRAB_BOTTLE", "GRAB"}:
                self._publish_arm_offsets()
            self._publish_arm_command(ARM_PLAIN_COMMANDS[plain_command])
            return

        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"机械臂指令格式错误：{exc}")
            return

        command_type = str(data.get("type", "")).strip().lower()
        if command_type in {"get_offsets", "query_offsets", "sync_offsets"}:
            # The bridge survives water-mode arm restarts and is the source of
            # truth. Reapply its saved values before reporting them to the app.
            self._publish_arm_offsets()
            self._publish_arm_offsets_status()
            return
        command_map = {
            "grab_bottle": "GRAB_BOTTLE",
            "vision_grab": "GRAB_BOTTLE",
            "auto_grab": "GRAB_BOTTLE",
            "stop": "STOP",
            "release": "RELEASE",
            "open": "RELEASE",
            "open_gripper": "RELEASE",
            "gripper_open": "RELEASE",
            "close": "CLOSE",
            "close_gripper": "CLOSE",
            "gripper_close": "CLOSE",
        }
        if command_type in command_map:
            if command_type in {"grab_bottle", "vision_grab", "auto_grab"}:
                self._publish_arm_offsets()
            self._publish_arm_command(command_map[command_type])
            return

        if command_type == "set_offsets":
            try:
                offsets = {
                    key: float(data[key])
                    for key in ("x_offset", "y_offset", "z_offset")
                }
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning("机械臂抓取偏移缺少有效的 X、Y、Z 数值")
                return
            if not all(
                math.isfinite(value) and -0.5 <= value <= 0.5
                for value in offsets.values()
            ):
                self.get_logger().warning("机械臂抓取偏移必须在 -0.5 到 0.5 米之间")
                return
            self.arm_offsets = offsets
            self._save_arm_offsets()
            self._publish_arm_offsets()
            self._publish_arm_offsets_status()
            return

        angles = data.get("angles")
        if not isinstance(angles, list) or len(angles) < 6:
            self.get_logger().warning("机械臂角度指令缺少 6 个关节数据")
            return

        try:
            positions = [float(value) for value in angles[:6]]
        except (TypeError, ValueError):
            self.get_logger().warning("机械臂关节角度不是有效数字")
            return

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f"joint_{index}" for index in range(1, 7)]
        joint_msg.position = positions
        self.arm_teleop_pub.publish(joint_msg)

    def _handle_esp32_status(self, payload):
        ros_msg = String()
        try:
            data = json.loads(payload)
            event = data.get("event", "STATUS")
            light = "ON" if data.get("light") else "OFF"
            fan = "ON" if data.get("fan") else "OFF"
            alarm = "ON" if data.get("alarm") else "OFF"
            ip_address = data.get("ip", "")
            ros_msg.data = (
                f"{event} | light={light} fan={fan} alarm={alarm} {ip_address}"
            ).strip()
        except (json.JSONDecodeError, TypeError, ValueError):
            ros_msg.data = payload
        self.esp32_status_pub.publish(ros_msg)
        self.get_logger().info(f"ESP32 状态回传：{ros_msg.data}")

    def _handle_patrol_command(self, payload):
        command = payload.strip().upper()
        is_json = False
        try:
            data = json.loads(payload)
            command = str(data.get("command", data.get("cmd", command))).upper()
            is_json = isinstance(data, dict)
        except (json.JSONDecodeError, AttributeError, TypeError):
            pass

        if command not in {
            "START",
            "START_PATH",
            "SET_ROOM",
            "GOTO_ROOM",
            "STOP",
            "RELOAD",
        }:
            self.get_logger().warning(f"不支持的家庭巡查指令：{command}")
            return
        msg = String()
        msg.data = (
            payload
            if command in {"START_PATH", "SET_ROOM", "GOTO_ROOM"} and is_json
            else command
        )
        self.patrol_cmd_pub.publish(msg)
        self.get_logger().info(f"家庭巡查指令：{command}")

    def _handle_navigation_goal(self, payload):
        try:
            data = json.loads(payload)
            room = str(data.get("room", "")).strip()
        except (json.JSONDecodeError, AttributeError, TypeError):
            room = payload.strip()

        if not room:
            self.get_logger().warning("家庭地图导航缺少房间名称")
            return

        msg = String()
        msg.data = json.dumps(
            {"command": "GOTO_ROOM", "room": room}, ensure_ascii=False
        )
        self.patrol_cmd_pub.publish(msg)
        self.get_logger().info(f"家庭地图导航目标：{room}")

    def _handle_navigation_system_command(self, payload):
        command = payload.strip().upper()
        try:
            data = json.loads(payload)
            command = str(data.get("command", data.get("cmd", command))).upper()
        except (json.JSONDecodeError, AttributeError, TypeError):
            pass
        if command not in {
            "START",
            "STOP",
            "STATUS",
            "RESET_ORIGIN",
            "SET_INITIAL_POSE",
            "SET_MODE",
            "MODE_INTERACTION",
            "MODE_WATER",
            "MODE_PATROL",
            "DEMO_INTRO_WEATHER",
            "DEMO_HOME_ENVIRONMENT",
            "ENABLE_VOICE_INTERACTION",
        }:
            self.get_logger().warning(f"不支持的导航系统指令：{command}")
            return
        msg = String()
        msg.data = payload
        self.navigation_system_cmd_pub.publish(msg)
        self.get_logger().info(f"导航系统指令：{command}")

    def _on_patrol_ros_status(self, msg):
        if not self.mqtt_connected:
            self.get_logger().warning("MQTT 尚未连接，巡查状态未回传")
            return
        result = self.mqtt_client.publish(
            self.patrol_status_topic, msg.data, retain=True
        )
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f"巡查状态回传失败，返回码：{result.rc}")

    def _on_navigation_system_status(self, msg):
        try:
            status = json.loads(msg.data)
            is_navigation_mode = str(status.get("mode", "")).lower() == "patrol"
        except (json.JSONDecodeError, AttributeError, TypeError):
            is_navigation_mode = self.navigation_mode

        if is_navigation_mode != self.navigation_mode:
            # 切换速度通道前同时清零旧、新通道，避免上一模式的速度残留。
            self.cmd_vel_pub.publish(Twist())
            self.nav_cmd_vel_pub.publish(Twist())
            self.navigation_mode = is_navigation_mode
            self.last_cmd_vel_time = 0.0
            self.brake_sent = True
            channel = "Nav2平滑防撞链" if is_navigation_mode else "底盘直连"
            self.get_logger().info(f"手机速度通道已切换：{channel}")

        if not self.mqtt_connected:
            self.get_logger().warning("MQTT 尚未连接，导航系统状态未回传")
            return
        result = self.mqtt_client.publish(
            self.navigation_system_status_topic, msg.data, retain=True
        )
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                f"导航系统状态回传失败，返回码：{result.rc}"
            )

    def _on_arm_ros_status(self, msg):
        if not self.mqtt_connected:
            self.get_logger().warning("MQTT 尚未连接，机械臂识别状态未回传")
            return
        message = msg.data.strip()
        if not message:
            return
        try:
            structured = json.loads(message)
        except (json.JSONDecodeError, TypeError):
            structured = None
        if (
            isinstance(structured, dict)
            and str(structured.get("event", "")).lower() == "arm_offsets"
        ):
            try:
                offsets = {
                    key: float(structured[key])
                    for key in ("x_offset", "y_offset", "z_offset")
                }
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning("机械臂回传的抓取参数格式无效")
                return
            self.arm_offsets = offsets
            self._save_arm_offsets()
            self._publish_arm_offsets_status()
            return
        if "未发现水瓶" in message or "离开画面" in message:
            event = "bottle_lost"
            message = ""
        elif message == "发现水瓶" or (
            "瓶子" in message and "可以" in message and "点击" in message
        ):
            event = "bottle_detected"
            message = "发现水瓶"
        elif "目标坐标平均完成" in message:
            event = "target_confirmed"
        elif "演示完成" in message or "抓取成功" in message:
            event = "grab_completed"
        elif "失败" in message or "异常" in message:
            event = "arm_error"
        else:
            event = "arm_status"
        payload = json.dumps(
            {
                "event": event,
                "message": message,
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            },
            ensure_ascii=False,
        )
        result = self.mqtt_client.publish(
            self.arm_status_topic, payload, retain=True
        )
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                f"机械臂识别状态回传失败，返回码：{result.rc}"
            )

    def _on_security_ros_alert(self, msg):
        if not self.mqtt_connected:
            self.get_logger().warning("MQTT 尚未连接，家庭安全提醒未发送")
            return
        result = self.mqtt_client.publish(self.security_alert_topic, msg.data)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().warning("家庭紧急求救已发送到手机 App")
        else:
            self.get_logger().error(f"家庭安全提醒发送失败，返回码：{result.rc}")

    def _on_esp32_ros_command(self, msg):
        command = msg.data.strip().upper()
        if not command:
            return
        if not self.mqtt_connected:
            self.get_logger().warning("MQTT 尚未连接，ESP32 指令未发送")
            return

        result = self.mqtt_client.publish(self.esp32_cmd_topic, command)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(f"ESP32 控制指令：{command}")
        else:
            self.get_logger().error(f"ESP32 指令发送失败，返回码：{result.rc}")

    def _watchdog(self):
        if self.brake_sent or self.last_cmd_vel_time <= 0.0:
            return
        if time.monotonic() - self.last_cmd_vel_time <= self.cmd_vel_timeout:
            return

        self._publish_drive_twist(Twist())
        self.brake_sent = True
        self.get_logger().warning(
            f"手机遥控超过 {self.cmd_vel_timeout:.1f} 秒无数据，底盘已自动刹车"
        )

    def destroy_node(self):
        self.shutting_down = True
        try:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnifiedMqttBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and rclpy.ok():
            node.get_logger().info("收到停止指令，正在关闭桥接节点")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
