#!/usr/bin/env python3
import json
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import serial
import paho.mqtt.client as mqtt


SERIAL_PORT = "/dev/air_sensor"
BAUD_RATE = 9600
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
PHONE_ALERT_TOPIC = "home/security/alert"
PHONE_ENV_TOPIC = "home/environment/state"


class AirSensorNode(Node):
    def __init__(self):
        super().__init__("air_sensor_node")

        self.publisher_ = self.create_publisher(String, "/air_sensor_data", 10)
        self.esp32_cmd_pub = self.create_publisher(String, "/esp32_cmd", 10)
        self.test_sub = self.create_subscription(
            String, "/air_alert_test", self.test_alert_callback, 10
        )

        self.alarm_active = False
        self.fan_active = False
        self.normal_count = 0
        self.last_phone_alert_level = None
        self.demo_override_until = 0.0
        self.demo_override_data = None

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "air_security_alert")
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"手机预警 MQTT 已连接: {PHONE_ALERT_TOPIC}")
        except Exception as exc:
            self.get_logger().warn(f"手机预警 MQTT 连接失败: {exc}")

        self.get_logger().info(f"环境传感器节点启动，正在连接 {SERIAL_PORT}...")

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

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

        # Demo thresholds. HCHO 100 ug/m3 ~= 0.1 mg/m3.
        if co2 > 2500:
            severe_reasons.append(f"CO2 {co2:.0f}ppm")
        elif co2 > 1800:
            warning_reasons.append(f"CO2 {co2:.0f}ppm")

        if jq > 100:
            severe_reasons.append(f"甲醛 {jq:.0f}ug/m3")
        elif jq > 60:
            warning_reasons.append(f"甲醛 {jq:.0f}ug/m3")

        if voc > 600:
            severe_reasons.append(f"VOC {voc:.0f}ug/m3")
        elif voc > 300:
            warning_reasons.append(f"VOC {voc:.0f}ug/m3")

        if pm25 > 75:
            severe_reasons.append(f"PM2.5 {pm25:.0f}ug/m3")
        elif pm25 > 35:
            warning_reasons.append(f"PM2.5 {pm25:.0f}ug/m3")

        if pm10 > 150:
            severe_reasons.append(f"PM10 {pm10:.0f}ug/m3")
        elif pm10 > 75:
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

    def publish_phone_alert(self, level, data, severe, warning):
        if level == "良好":
            if self.last_phone_alert_level != "良好":
                payload = {
                    "type": "AIR_SECURITY_CLEAR",
                    "level": "良好",
                    "title": "空气安全恢复正常",
                    "message": "当前空气质量恢复正常，蜂鸣器与通风风扇已关闭。",
                    "location": "家庭环境",
                    "data": data,
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
            "data": data,
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
            "voc": data.get("VOC"),
            "hcho": data.get("甲醛"),
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

    def handle_linkage(self, level, data, severe, warning):
        self.publish_phone_alert(level, data, severe, warning)

        if level == "异常":
            self.normal_count = 0
            if not self.alarm_active:
                self.send_esp32_cmd("ALARM_ON")
                self.alarm_active = True
            if not self.fan_active:
                self.send_esp32_cmd("FAN_ON")
                self.fan_active = True
        elif level == "一般":
            self.normal_count = 0
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
                if self.alarm_active:
                    self.send_esp32_cmd("ALARM_OFF")
                    self.alarm_active = False
                if self.fan_active:
                    self.send_esp32_cmd("FAN_OFF")
                    self.fan_active = False

    def publish_air_data(self, result_dict):
        level, advice, severe, warning = self.evaluate_air(result_dict)
        result_dict["等级"] = level
        result_dict["建议"] = advice
        result_dict["异常项"] = severe
        result_dict["提醒项"] = warning

        msg = String()
        msg.data = json.dumps(result_dict, ensure_ascii=False)
        self.publisher_.publish(msg)

        self.publish_environment_state(result_dict)
        self.handle_linkage(level, result_dict, severe, warning)

    def demo_alert_data(self):
        return {
            "CO2": 1500,
            "甲醛": 130,
            "VOC": 850,
            "PM2.5": 90,
            "PM10": 170,
            "温度": 26.5,
            "湿度": 58.0,
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
        elif cmd in ("OFF", "ALARM_OFF", "TEST_OFF", "CLEAR"):
            normal = self.demo_normal_data()
            self.demo_override_data = None
            self.demo_override_until = 0.0
            self.last_phone_alert_level = None
            self.get_logger().info("清除演示用空气安全异常")
            self.normal_count = 5
            self.publish_air_data(normal)

    def read_serial_loop(self):
        while rclpy.ok():
            ser = None
            try:
                ser = serial.Serial(
                    SERIAL_PORT,
                    BAUD_RATE,
                    timeout=1,
                    exclusive=True,
                )
                ser.reset_input_buffer()
                self.get_logger().info("串口连接成功，开始广播空气质量数据...")

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
