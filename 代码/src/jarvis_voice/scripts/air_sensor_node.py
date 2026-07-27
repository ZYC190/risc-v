#!/usr/bin/env python3
import json
import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import serial
import paho.mqtt.client as mqtt


SERIAL_PORT = "/dev/air_sensor"
SERIAL_FALLBACK_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
)
BAUD_RATE = 9600
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
PHONE_ALERT_TOPIC = "home/security/alert"
PHONE_ENV_TOPIC = "home/environment/state"
PHONE_ENV_REQUEST_TOPIC = "home/environment/request"
PROBLEM_CONFIRM_FRAMES = 3
LIVE_SAMPLE_MAX_AGE = 8.0


def resolve_serial_port():
    """Prefer the udev alias, with a stable CH340 by-id fallback.

    The air sensor is the only 1a86:7523 CH340 on this robot.  The fallback
    deliberately does not scan arbitrary ttyUSB devices, so it cannot select
    the CP2102N lidar by mistake.
    """
    if os.path.exists(SERIAL_PORT):
        return SERIAL_PORT
    if os.path.exists(SERIAL_FALLBACK_PORT):
        return SERIAL_FALLBACK_PORT
    return SERIAL_PORT


class AirSensorNode(Node):
    def __init__(self):
        super().__init__("air_sensor_node")
        self.declare_parameter("linkage_enabled", True)

        self.publisher_ = self.create_publisher(String, "/air_sensor_data", 10)
        self.esp32_cmd_pub = self.create_publisher(String, "/esp32_cmd", 10)
        self.voice_announce_pub = self.create_publisher(String, "/voice_announce", 10)
        self.test_sub = self.create_subscription(
            String, "/air_alert_test", self.test_alert_callback, 10
        )
        self.refresh_sub = self.create_subscription(
            String, "/air_sensor_refresh", self.refresh_callback, 10
        )
        self.control_sub = self.create_subscription(
            String, "/air_alert_control", self.control_callback, 10
        )

        self.alarm_active = False
        self.fan_active = False
        self.light_active = False
        self.normal_count = 0
        self.problem_count = 0
        self.pending_problem_level = None
        self.last_phone_alert_level = None
        self.demo_override_until = 0.0
        self.demo_override_data = None
        self.last_demo_voice_state = None
        self.linkage_enabled = bool(
            self.get_parameter("linkage_enabled").value
        )
        self.latest_sample = None
        self.latest_sample_time = 0.0
        self.latest_sample_lock = threading.Lock()

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "air_security_alert")
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"手机预警 MQTT 已连接: {PHONE_ALERT_TOPIC}")
        except Exception as exc:
            self.get_logger().warn(f"手机预警 MQTT 连接失败: {exc}")

        startup_port = resolve_serial_port()
        if startup_port != SERIAL_PORT:
            self.get_logger().warning(
                f"{SERIAL_PORT} 不存在，已安全回退到空气传感器固定设备 {startup_port}"
            )
        self.get_logger().info(f"环境传感器节点启动，正在连接 {startup_port}...")
        if not self.linkage_enabled:
            self.get_logger().info("环境监测保持运行，声光/风扇自动联动已关闭")

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(PHONE_ENV_REQUEST_TOPIC)
            self.get_logger().info(
                f"环境刷新 MQTT 已订阅: {PHONE_ENV_REQUEST_TOPIC}"
            )
        else:
            self.get_logger().warning(f"环境刷新 MQTT 连接返回码异常: {rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        if msg.topic == PHONE_ENV_REQUEST_TOPIC:
            self.publish_latest_sample("手机 App")

    def decode_airmod(self, data):
        if len(data) != 17:
            return None

        checksum = sum(data[0:16]) & 0xFF
        if not (data[0] == 0x3C and data[1] == 0x02 and checksum == data[16]):
            return None

        co2 = (data[2] << 8) | data[3]
        jq = (data[4] << 8) | data[5]
        voc = (data[6] << 8) | data[7]
        pm25 = (data[8] << 8) | data[9]
        pm10 = (data[10] << 8) | data[11]

        temp_minus = 1 if (data[12] & 0x80) else 0
        temp_float = (data[12] & 0x7F) + (data[13] * 0.1)
        if temp_minus:
            temp_float = -temp_float

        humi_float = data[14] + (data[15] * 0.1)

        return {
            "CO2": co2,
            "甲醛": jq,
            "VOC": voc,
            "PM2.5": pm25,
            "PM10": pm10,
            "温度": round(temp_float, 1),
            "湿度": round(humi_float, 1),
        }

    def evaluate_air(self, data):
        """Return (level, advice, severe_reasons, warning_reasons)."""
        severe_reasons = []
        warning_reasons = []

        co2 = float(data.get("CO2", 0))
        jq = float(data.get("甲醛", 0))
        voc = float(data.get("VOC", 0))
        pm25 = float(data.get("PM2.5", 0))
        pm10 = float(data.get("PM10", 0))

        # Competition thresholds intentionally leave a wide margin for the
        # low-cost module's warm-up drift. CO2 remains display-only.
        if jq > 300:
            severe_reasons.append(f"甲醛 {jq:.0f}ug/m3")
        elif jq > 200:
            warning_reasons.append(f"甲醛 {jq:.0f}ug/m3")

        if voc > 3000:
            severe_reasons.append(f"VOC {voc:.0f}ug/m3")
        elif voc > 1800:
            warning_reasons.append(f"VOC {voc:.0f}ug/m3")

        if pm25 > 500:
            severe_reasons.append(f"PM2.5 {pm25:.0f}ug/m3")
        elif pm25 > 250:
            warning_reasons.append(f"PM2.5 {pm25:.0f}ug/m3")

        if pm10 > 800:
            severe_reasons.append(f"PM10 {pm10:.0f}ug/m3")
        elif pm10 > 400:
            warning_reasons.append(f"PM10 {pm10:.0f}ug/m3")

        if severe_reasons:
            return "异常", "空气质量异常，蜂鸣器与通风风扇已触发，请立即通风并远离异常区域", severe_reasons, warning_reasons
        if warning_reasons:
            return "一般", "空气质量一般，已开启辅助通风并继续观察", severe_reasons, warning_reasons
        return "良好", "空气质量良好", severe_reasons, warning_reasons

    def send_esp32_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.esp32_cmd_pub.publish(msg)
        self.get_logger().info(f"联动 ESP32: {cmd}")

    def announce(self, text):
        msg = String()
        msg.data = text
        self.voice_announce_pub.publish(msg)
        self.get_logger().info(f"请求机器人安全播报: {text}")

    def publish_phone_alert(self, level, data, severe, warning):
        phone_data = dict(data)
        phone_data["甲醛"] = self.ug_to_mg(data.get("甲醛"))
        phone_data["VOC"] = self.ug_to_mg(data.get("VOC"))
        if level == "良好":
            if self.last_phone_alert_level is None:
                self.last_phone_alert_level = "良好"
                return
            if self.last_phone_alert_level != "良好":
                payload = {
                    "type": "AIR_SECURITY_CLEAR",
                    "level": "良好",
                    "title": "空气安全恢复正常",
                    "message": "当前空气质量恢复正常，蜂鸣器与通风风扇已关闭。",
                    "location": "家庭环境",
                    "data": phone_data,
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                }
                self.mqtt_client.publish(
                    PHONE_ALERT_TOPIC,
                    json.dumps(payload, ensure_ascii=False),
                    retain=False,
                )
                self.last_phone_alert_level = "良好"
            return

        # Avoid sending the same warning every second.
        if self.last_phone_alert_level == level:
            return

        reasons = severe or warning
        payload = {
            "type": "AIR_SECURITY_ALERT",
            "level": level,
            "title": "家庭空气安全预警",
            "message": "；".join(reasons) if reasons else "空气数据异常",
            "location": "家庭环境",
            "suggestion": "请立即通风，远离异常区域，并检查空气来源。",
            "data": phone_data,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.mqtt_client.publish(
            PHONE_ALERT_TOPIC,
            json.dumps(payload, ensure_ascii=False),
            retain=False,
        )
        self.last_phone_alert_level = level
        self.get_logger().warn(f"手机预警已发送: {payload['message']}")

    def publish_environment_state(self, data):
        payload = {
            "type": "ENVIRONMENT_STATE",
            "temperature": data.get("温度"),
            "humidity": data.get("湿度"),
            "co2": data.get("CO2"),
            "pm25": data.get("PM2.5"),
            "pm10": data.get("PM10"),
            "voc": self.ug_to_mg(data.get("VOC")),
            "hcho": self.ug_to_mg(data.get("甲醛")),
            "level": data.get("等级", "未知"),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        try:
            self.mqtt_client.publish(
                PHONE_ENV_TOPIC,
                json.dumps(payload, ensure_ascii=False),
                retain=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"环境实时数据 MQTT 发布失败: {exc}")

    @staticmethod
    def ug_to_mg(value):
        try:
            return round(float(value) / 1000.0, 3)
        except (TypeError, ValueError):
            return value

    def handle_linkage(self, level, data, severe, warning):
        if not self.linkage_enabled:
            self.problem_count = 0
            self.pending_problem_level = None
            return

        demo_forced = (
            self.demo_override_data is not None
            and time.time() < self.demo_override_until
        )

        if level != "良好" and not demo_forced:
            if self.pending_problem_level == level:
                self.problem_count += 1
            else:
                self.pending_problem_level = level
                self.problem_count = 1

            if self.problem_count < PROBLEM_CONFIRM_FRAMES:
                self.get_logger().info(
                    f"检测到空气{level}，等待连续确认 "
                    f"{self.problem_count}/{PROBLEM_CONFIRM_FRAMES}"
                )
                return
        else:
            self.problem_count = 0
            self.pending_problem_level = None

        self.publish_phone_alert(level, data, severe, warning)

        if level == "异常":
            self.normal_count = 0
            if not self.light_active:
                self.send_esp32_cmd("LIGHT_ON")
                self.light_active = True
            if not self.alarm_active:
                self.send_esp32_cmd("ALARM_ON")
                self.alarm_active = True
            if not self.fan_active:
                self.send_esp32_cmd("FAN_ON")
                self.fan_active = True
        elif level == "一般":
            self.normal_count = 0
            if self.light_active:
                self.send_esp32_cmd("LIGHT_OFF")
                self.light_active = False
            if self.alarm_active:
                self.send_esp32_cmd("ALARM_OFF")
                self.alarm_active = False
            if not self.fan_active:
                self.send_esp32_cmd("FAN_ON")
                self.fan_active = True
        else:
            # Require several normal frames before turning devices off.
            self.normal_count += 1
            if self.normal_count >= 5:
                if self.light_active:
                    self.send_esp32_cmd("LIGHT_OFF")
                    self.light_active = False
                if self.alarm_active:
                    self.send_esp32_cmd("ALARM_OFF")
                    self.alarm_active = False
                if self.fan_active:
                    self.send_esp32_cmd("FAN_OFF")
                    self.fan_active = False

    def publish_air_data(self, result_dict):
        result_dict = dict(result_dict)
        level, advice, severe, warning = self.evaluate_air(result_dict)
        result_dict["等级"] = level
        result_dict["建议"] = advice
        result_dict["异常项"] = severe
        result_dict["提醒项"] = warning

        msg = String()
        msg.data = json.dumps(result_dict, ensure_ascii=False)
        self.publisher_.publish(msg)

        with self.latest_sample_lock:
            self.latest_sample = dict(result_dict)
            self.latest_sample_time = time.monotonic()

        self.publish_environment_state(result_dict)
        self.handle_linkage(level, result_dict, severe, warning)

    def publish_latest_sample(self, requester):
        with self.latest_sample_lock:
            sample = dict(self.latest_sample) if self.latest_sample else None
            sample_age = time.monotonic() - self.latest_sample_time

        if sample is None or sample_age > LIVE_SAMPLE_MAX_AGE:
            self.get_logger().warning(
                f"{requester}请求刷新，但最近 {LIVE_SAMPLE_MAX_AGE:.0f} 秒没有传感器采样"
            )
            return

        msg = String()
        msg.data = json.dumps(sample, ensure_ascii=False)
        self.publisher_.publish(msg)
        self.publish_environment_state(sample)
        self.get_logger().info(f"已向{requester}刷新最新环境数据")

    def refresh_callback(self, msg):
        self.publish_latest_sample("触摸屏")

    def control_callback(self, msg):
        command = msg.data.strip().upper()
        if command in ("LINKAGE_OFF", "DISABLE", "PAUSE"):
            self.linkage_enabled = False
            self.demo_override_data = None
            self.demo_override_until = 0.0
            self.last_phone_alert_level = None
            self.problem_count = 0
            self.pending_problem_level = None
            if self.alarm_active:
                self.send_esp32_cmd("ALARM_OFF")
                self.alarm_active = False
            if self.fan_active:
                self.send_esp32_cmd("FAN_OFF")
                self.fan_active = False
            if self.light_active:
                self.send_esp32_cmd("LIGHT_OFF")
                self.light_active = False
            self.get_logger().info("导航模式：空气预警与 ESP32 自动联动已暂停")
        elif command in ("LINKAGE_ON", "ENABLE", "RESUME"):
            self.linkage_enabled = True
            self.last_phone_alert_level = None
            self.normal_count = 0
            self.get_logger().info("演示模式：空气预警与 ESP32 自动联动已恢复")

    def demo_alert_data(self):
        return {
            "CO2": 1500,
            "甲醛": 350,
            "VOC": 3500,
            "PM2.5": 550,
            "PM10": 850,
            "温度": 26.5,
            "湿度": 58.0,
        }

    def demo_hcho_alert_data(self):
        return {
            "CO2": 550,
            "甲醛": 350,
            "VOC": 80,
            "PM2.5": 12,
            "PM10": 25,
            "温度": 26.0,
            "湿度": 55.0,
        }

    def demo_normal_data(self):
        return {
            "CO2": 550,
            "甲醛": 20,
            "VOC": 80,
            "PM2.5": 12,
            "PM10": 25,
            "温度": 26.0,
            "湿度": 55.0,
        }

    def test_alert_callback(self, msg):
        cmd = msg.data.strip().upper()
        if cmd in ("ON", "ALARM_ON", "TEST_ON"):
            fake = self.demo_alert_data()
            self.demo_override_data = fake
            self.demo_override_until = time.time() + 30
            self.last_phone_alert_level = None
            self.get_logger().warn("触发演示用空气安全异常：触摸屏和手机将同时预警，保持 30 秒")
            self.publish_air_data(fake)
        elif cmd in ("HCHO_ON", "FORMALDEHYDE_ON", "JQ_ON"):
            should_announce = self.last_demo_voice_state != "HCHO_ON"
            fake = self.demo_hcho_alert_data()
            self.demo_override_data = fake
            # Keep the formaldehyde demo active until the operator explicitly
            # publishes CLEAR.  Real sensor frames must not end this demo.
            self.demo_override_until = float("inf")
            self.last_phone_alert_level = None
            self.get_logger().warn(
                "触发甲醛单项超标演示：甲醛 350ug/m3，持续保持，等待 CLEAR 恢复"
            )
            self.publish_air_data(fake)
            if should_announce:
                self.announce("甲醛数据超标，请远离，已开启通风。")
            self.last_demo_voice_state = "HCHO_ON"
        elif cmd in ("OFF", "ALARM_OFF", "TEST_OFF", "CLEAR"):
            should_announce = self.last_demo_voice_state != "CLEAR"
            normal = self.demo_normal_data()
            self.demo_override_data = None
            self.demo_override_until = 0.0
            self.last_phone_alert_level = None
            self.get_logger().info("清除演示用空气安全异常")
            self.normal_count = 5
            self.publish_air_data(normal)
            if should_announce:
                self.announce("甲醛数据恢复正常，关闭通风。")
            self.last_demo_voice_state = "CLEAR"

    def read_serial_loop(self):
        while rclpy.ok():
            ser = None
            serial_port = resolve_serial_port()
            try:
                ser = serial.Serial(
                    serial_port,
                    BAUD_RATE,
                    timeout=1,
                    exclusive=True,
                )
                ser.reset_input_buffer()
                self.get_logger().info(
                    f"串口 {serial_port} 连接成功，开始广播空气质量数据..."
                )

                while rclpy.ok():
                    try:
                        first = ser.read(1)
                        if first != b"\x3C":
                            continue

                        second = ser.read(1)
                        if second != b"\x02":
                            continue

                        rest = ser.read(15)
                        if len(rest) != 15:
                            self.get_logger().warn(
                                f"空气传感器数据帧不完整: {len(rest)}/15，继续等待下一帧"
                            )
                            continue

                        frame = b"\x3C\x02" + rest
                        result_dict = self.decode_airmod(frame)
                        if result_dict:
                            if self.demo_override_data and time.time() < self.demo_override_until:
                                result_dict = dict(self.demo_override_data)
                            elif self.demo_override_data and time.time() >= self.demo_override_until:
                                self.get_logger().info("演示异常保持时间结束，恢复真实传感器数据")
                                self.demo_override_data = None
                                self.demo_override_until = 0.0
                            self.publish_air_data(result_dict)

                        time.sleep(0.5)
                    except (serial.SerialException, OSError) as exc:
                        self.get_logger().warn(
                            f"串口通信中断: {exc}，3 秒后自动重连..."
                        )
                        break
            except Exception as exc:
                self.get_logger().warn(
                    f"串口连接失败: {exc}，3 秒后自动重试..."
                )
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

            time.sleep(3)

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
