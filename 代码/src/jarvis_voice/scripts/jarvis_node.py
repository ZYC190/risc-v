#!/usr/bin/env python3
"""
融合版 jarvis_node：声源定位 + 唤醒词 + 机械臂控制 + DeepSeek聊天
- 默认麦克风开启，说一次“小薇/小微”或常见近音词即可唤醒
- 识别到唤醒词后 → 获取声源角度 → 小车转向唤醒者 → 进入聊天/控制处理
- 家庭服务机器人名字叫“小微”
- GUI 语音页面按钮可开启/关闭麦克风
- 机械臂动作指令始终可用（唤醒后执行）
"""
import os
os.environ["PA_ALSA_PLUGHW"] = "1"

import re
import tempfile
import datetime
import threading
import queue
import time
import math
import requests
import json
import base64
import subprocess
import numpy as np

try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

from pydub import AudioSegment
from pydub.playback import play
from openai import OpenAI
import speech_recognition as sr
from aip import AipSpeech

# ==========================================
# API 密钥配置：优先读取环境变量；也可在仓库根目录创建 .robot_secrets
# ==========================================
def _load_local_secrets():
    candidates = [
        os.environ.get("ROBOT_SECRETS_FILE", ""),
        os.path.expanduser("~/.robot_secrets"),
        os.path.expanduser("~/robot2/.robot_secrets"),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".robot_secrets")),
    ]
    for path in candidates:
        if not path or not os.path.exists(path):
            continue
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


_load_local_secrets()

DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")

BAIDU_APP_ID = os.environ.get("BAIDU_APP_ID", "")
BAIDU_API_KEY = os.environ.get("BAIDU_API_KEY", "")
BAIDU_SECRET_KEY = os.environ.get("BAIDU_SECRET_KEY", "")

GAODE_API_KEY = os.environ.get("GAODE_API_KEY", "")
CITY_CODE = "500000"
# ==========================================

# 初始化全局云端客户端
client_llm = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")
BAIDU_TOKEN = None
chat_history = [{"role": "system", "content": "初始化人设"}]

# 声源角度（由ROS2回调更新）
audio_angle = 0
last_angle = -999
audio_angle_valid = False
audio_awake_flag = 0
angle_lock = threading.Lock()

WAKE_ALIASES = (
    "小薇",
    "小微",
    "小威",
    "小伟",
    "小维",
    "小唯",
    "小卫",
    "晓薇",
    "晓微",
    "小为",
)

# 里程计当前朝向
current_yaw = 0.0
odom_lock = threading.Lock()


# --- 全局工具函数 ---
def get_baidu_token():
    global BAIDU_TOKEN
    if BAIDU_TOKEN is None:
        url = f"https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id={BAIDU_API_KEY}&client_secret={BAIDU_SECRET_KEY}"
        try:
            res = requests.post(url)
            BAIDU_TOKEN = res.json().get("access_token")
        except Exception as e:
            print(f"❌ 获取百度 Token 失败: {e}")
    return BAIDU_TOKEN


def get_weather():
    if not GAODE_API_KEY:
        return "指挥官暂未配置气象雷达密钥，无法获取天气。"
    url = f"https://restapi.amap.com/v3/weather/weatherInfo?city={CITY_CODE}&key={GAODE_API_KEY}"
    try:
        res = requests.get(url, timeout=3).json()
        if res.get("status") == "1" and len(res.get("lives", [])) > 0:
            w = res["lives"][0]
            return f"{w['city']}天气{w['weather']}，气温{w['temperature']}度，{w['winddirection']}风{w['windpower']}级。"
    except Exception:
        pass
    return "气象卫星连接失败。"


def remove_think_tag(text):
    return re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL)


def extract_wake_command(text):
    """Return text after one Xiaowei-like prefix, or None if not woken."""
    normalized = re.sub(r"[\s，。！？、,.!?：:；;～~]+", "", str(text or ""))
    for wake_prefix in WAKE_ALIASES:
        if not normalized.startswith(wake_prefix):
            continue
        command = normalized[len(wake_prefix):]
        # 兼容原来的“小薇小薇”，但不再强制必须重复呼叫。
        for repeated_prefix in WAKE_ALIASES:
            if command.startswith(repeated_prefix):
                command = command[len(repeated_prefix):]
                break
        return command
    return None


def baidu_tts(text):
    """百度云端语音合成，返回临时WAV文件路径"""
    token = get_baidu_token()
    if not token:
        return None
    url = "https://tsn.baidu.com/text2audio"
    payload = {
        'tex': text, 'tok': token, 'cuid': 'agv_car_001',
        'ctp': 1, 'lan': 'zh', 'spd': 5, 'pit': 5,
        'vol': 15, 'per': 4, 'aue': 6
    }
    try:
        res = requests.post(url, data=payload)
        if res.headers.get('Content-Type') == 'audio/wav':
            temp_wav_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
            temp_wav_file.write(res.content)
            temp_wav_file.close()
            return temp_wav_file.name
        else:
            print(f"❌ 百度 TTS 报错: {res.text}")
            return None
    except Exception as e:
        print(f"❌ 百度 API 请求异常: {e}")
        return None


def play_audio_file(audio_file):
    """播放音频（重采样48000Hz + 放大15dB）"""
    try:
        if os.path.exists(audio_file):
            audio = AudioSegment.from_file(audio_file, parameters=["-loglevel", "quiet"])
            fixed_audio = (audio + 15).set_frame_rate(48000)
            temp_play_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
            temp_play_file.close()
            fixed_audio.export(temp_play_file.name, format="wav")
            try:
                subprocess.run(
                    ["aplay", "-q", temp_play_file.name],
                    check=True,
                    timeout=25,
                )
            except Exception:
                play(fixed_audio)
            finally:
                try:
                    os.unlink(temp_play_file.name)
                except OSError:
                    pass
        else:
            print(f'❌ 找不到音频文件: {audio_file}')
    except Exception as e:
        print(f"❌ 播放音频时出错: {e}")


# ==========================================
# 🤖 核心 ROS 2 节点类（融合声源定位+唤醒词+机械臂+聊天）
# ==========================================
class JarvisCommander(Node):
    def __init__(self):
        super().__init__('jarvis_commander_node')

        # ------ 机械臂控制 ------
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # ------ 声源定位 ------
        # 订阅声源角度话题
        self.angle_sub = self.create_subscription(
            Int32MultiArray, 'angle_topic', self.angle_callback, 10
        )
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        # 发布速度指令（用于转向声源）
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 20)

        # ------ 百度语音识别 ------
        self.client_asr = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
        self.care_dialogue_topic = "home/care/dialogue"
        self.parent_talk_topic = "home/care/parent_talk"
        self.mqtt_client = None
        self._init_mqtt_bridge()

        # ------ GUI 联动 ------
        self.voice_log_pub = self.create_publisher(String, '/voice_log', 10)
        self.voice_trigger_sub = self.create_subscription(
            String, '/voice_trigger', self.voice_trigger_callback, 10)

        # ------ 七合一室内环境传感器 ------
        self.air_data_lock = threading.Lock()
        self.latest_air_data = None
        self.latest_air_data_time = 0.0
        self.air_data_max_age = float(os.environ.get("JARVIS_AIR_DATA_MAX_AGE", "30"))
        self.air_sensor_sub = self.create_subscription(
            String, '/air_sensor_data', self.air_sensor_callback, 10)

        # ------ 麦克风默认开启 ------
        self.is_listening = True
        self.remote_talk_active = threading.Event()
        self.speech_lock = threading.Lock()
        self.ignore_mic_until = 0.0
        self.parent_talk_queue = queue.Queue()
        self.audio_queue = queue.Queue(maxsize=5)
        self.stop_background_listening = None
        self.voice_min_rms = int(os.environ.get("JARVIS_VOICE_MIN_RMS", "420"))
        self.voice_min_ms = int(os.environ.get("JARVIS_VOICE_MIN_MS", "240"))
        self.voice_snr_ratio = float(os.environ.get("JARVIS_VOICE_SNR_RATIO", "1.8"))

        self.get_logger().info("🚀 融合版 Jarvis 启动！声源定位 + 唤醒词 + 机械臂 + 聊天")
        self.get_logger().info("🔗 GUI 语音面板联动已就绪 (/voice_log + /voice_trigger)")
        self.get_logger().info("🌿 已订阅室内环境数据 /air_sensor_data")
        self.get_logger().info("🎤 麦克风默认开启，说“小薇”或相近称呼即可唤醒。")
        self.get_logger().info(
            "🛡️ 人声门控已开启: "
            f"最低RMS={self.voice_min_rms}, 持续时间={self.voice_min_ms}ms, "
            f"信噪比={self.voice_snr_ratio:.1f}"
        )

        self.parent_talk_thread = threading.Thread(target=self._parent_talk_worker)
        self.parent_talk_thread.daemon = True
        self.parent_talk_thread.start()

        # 启动后台监听线程
        self.listen_thread = threading.Thread(target=self.listen_and_act)
        self.listen_thread.daemon = True
        self.listen_thread.start()

        # 等待线程启动后通知 GUI 初始状态
        time.sleep(0.3)
        self._send_voice_log("STATUS:ON")

    def _init_mqtt_bridge(self):
        if mqtt is None:
            self.get_logger().warning("⚠️ 未安装 paho-mqtt，现场对话不会同步到手机 App")
            return
        try:
            self.mqtt_client = mqtt.Client(client_id="jarvis_care_dialogue")
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_message = self._on_mqtt_message
            self.mqtt_client.connect("127.0.0.1", 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"📲 现场对话同步已接入 MQTT: {self.care_dialogue_topic}")
        except Exception as e:
            self.mqtt_client = None
            self.get_logger().warning(f"⚠️ MQTT 对话同步连接失败: {e}")

    def _on_mqtt_connect(self, client, userdata, flags, rc, *args):
        if rc == 0:
            client.subscribe(self.parent_talk_topic)
            self.get_logger().info(f"📥 已订阅家长远程发话: {self.parent_talk_topic}")
        else:
            self.get_logger().warning(f"⚠️ MQTT 连接返回码异常: {rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        if msg.topic != self.parent_talk_topic:
            return
        payload = msg.payload.decode("utf-8", errors="ignore")
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            data = payload

        if isinstance(data, dict) and str(data.get("type", "")).lower() == "parent_audio":
            encoded_audio = str(data.get("audio_base64", "")).strip()
            if not encoded_audio or len(encoded_audio) > 2_000_000:
                self.get_logger().warning("⚠️ 家长录音为空或超过大小限制，已忽略")
                return
            try:
                duration_ms = max(0, int(data.get("duration_ms", 0) or 0))
            except (TypeError, ValueError):
                duration_ms = 0
            self.parent_talk_queue.put(
                {
                    "type": "audio",
                    "audio_base64": encoded_audio,
                    "mime": str(data.get("mime", "audio/mp4")),
                    "duration_ms": duration_ms,
                }
            )
            return

        text = str(data.get("text", "")).strip() if isinstance(data, dict) else str(data).strip()
        if not text:
            return
        self.parent_talk_queue.put({"type": "text", "text": text})

    def _parent_talk_worker(self):
        while True:
            message = self.parent_talk_queue.get()
            try:
                self._handle_parent_talk(message)
            except Exception as e:
                self.get_logger().warning(f"⚠️ 家长发话播报异常: {e}")
            finally:
                self.parent_talk_queue.task_done()

    def _handle_parent_talk(self, message):
        old_listening_state = self.is_listening
        self.remote_talk_active.set()
        self.is_listening = False
        self.ignore_mic_until = time.time() + 30.0
        try:
            self._send_voice_log("🔇 家长远程发话处理中，临时暂停现场麦克风")
            if message.get("type") == "audio":
                duration_ms = max(0, int(message.get("duration_ms", 0)))
                duration_text = f"{duration_ms / 1000.0:.1f}秒" if duration_ms else ""
                audio_bytes = base64.b64decode(
                    message.get("audio_base64", ""), validate=True
                )
                if not audio_bytes or len(audio_bytes) > 1_500_000:
                    raise ValueError("家长录音为空或超过 1.5MB")
                suffix = ".m4a" if "mp4" in message.get("mime", "") else ".aac"
                temp_audio = tempfile.NamedTemporaryFile(delete=False, suffix=suffix)
                temp_audio.write(audio_bytes)
                temp_audio.close()
                try:
                    self.get_logger().info(
                        f"📱 收到家长原声录音 {duration_text}，正在通过机器人播放"
                    )
                    self._send_voice_log(f"📱 家长原声 {duration_text}")
                    self._publish_care_dialogue(
                        "parent", f"[家长原声 {duration_text}]".strip()
                    )
                    self._send_voice_log("📣 正在播放家长原声")
                    play_audio_file(temp_audio.name)
                finally:
                    try:
                        os.unlink(temp_audio.name)
                    except OSError:
                        pass
            else:
                text = str(message.get("text", "")).strip()
                if not text:
                    return
                self.get_logger().info(f"📱 家长远程发话: {text}")
                self._send_voice_log(f"📱 家长: {text}")
                self._publish_care_dialogue("parent", text)
                self._send_voice_log("📣 正在播报家长原话")
                self.speak_and_play(text, publish_dialogue=False)
        finally:
            self.ignore_mic_until = time.time() + 2.0
            self.remote_talk_active.clear()
            self.is_listening = old_listening_state
            self._send_voice_log("🎤 现场麦克风已恢复")

    def _publish_care_dialogue(self, role, text):
        clean_text = remove_think_tag(str(text)).strip()
        if not clean_text or self.mqtt_client is None:
            return
        payload = {
            "role": role,
            "text": clean_text,
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        try:
            self.mqtt_client.publish(
                self.care_dialogue_topic,
                json.dumps(payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
        except Exception as e:
            self.get_logger().warning(f"⚠️ 现场对话同步失败: {e}")

    # ==================== 室内环境数据 ====================
    def air_sensor_callback(self, msg):
        """缓存七合一传感器发布的最新室内环境数据。"""
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("环境数据不是 JSON 对象")
        except Exception as exc:
            self.get_logger().warning(f"⚠️ 室内环境数据解析失败: {exc}")
            return

        with self.air_data_lock:
            first_packet = self.latest_air_data is None
            self.latest_air_data = data
            self.latest_air_data_time = time.time()

        if first_packet:
            self.get_logger().info("✅ 已收到七合一传感器实时数据，可进行语音查询")
            self._send_voice_log("✅ 室内环境传感器已连接")

    @staticmethod
    def format_sensor_value(value, digits=1):
        if value is None:
            return None
        try:
            number = float(value)
            if number.is_integer():
                return str(int(number))
            return f"{number:.{digits}f}"
        except (TypeError, ValueError):
            return str(value)

    def get_indoor_environment_reply(self, query):
        """根据用户问题，用最新七合一传感器数据生成播报内容。"""
        with self.air_data_lock:
            data = dict(self.latest_air_data) if self.latest_air_data else None
            data_age = time.time() - self.latest_air_data_time

        if data is None:
            return "我还没有收到室内环境传感器数据，请先启动空气传感器节点。"
        if data_age > self.air_data_max_age:
            return f"室内环境数据已经超过{int(data_age)}秒没有更新，请检查传感器连接。"

        clean_query = re.sub(r'\s+', '', query).lower()
        temperature = self.format_sensor_value(data.get("温度"))
        humidity = self.format_sensor_value(data.get("湿度"))
        co2 = self.format_sensor_value(data.get("CO2"))
        hcho = self.format_sensor_value(data.get("甲醛"))
        voc = self.format_sensor_value(data.get("VOC"))
        pm25 = self.format_sensor_value(data.get("PM2.5"))
        pm10 = self.format_sensor_value(data.get("PM10"))
        level = str(data.get("等级", "未知"))
        advice = str(data.get("建议", "")).strip()

        if "温度" in clean_query or "湿度" in clean_query:
            values = []
            if temperature is not None:
                values.append(f"温度{temperature}摄氏度")
            if humidity is not None:
                values.append(f"湿度{humidity}%")
            return "当前室内" + "，".join(values) + f"，空气质量{level}。"

        if "二氧化碳" in clean_query or "co2" in clean_query:
            return f"当前室内二氧化碳浓度为{co2}ppm，空气质量{level}。{advice}"

        if "甲醛" in clean_query or "voc" in clean_query:
            return (
                f"当前甲醛为{hcho}微克每立方米，VOC为{voc}，"
                f"空气质量{level}。{advice}"
            )

        if "pm" in clean_query or "颗粒物" in clean_query or "粉尘" in clean_query:
            return (
                f"当前PM2.5为{pm25}，PM10为{pm10}，"
                f"空气质量{level}。{advice}"
            )

        summary = (
            f"当前室内温度{temperature}摄氏度，湿度{humidity}%，"
            f"二氧化碳{co2}ppm，PM2.5为{pm25}，甲醛{hcho}微克每立方米，"
            f"空气质量{level}。"
        )
        if advice and advice not in summary:
            summary += advice
        return summary

    # ==================== 声源定位回调 ====================
    def angle_callback(self, msg):
        global audio_angle, last_angle, audio_angle_valid, audio_awake_flag
        if msg.data and len(msg.data) >= 1:
            with angle_lock:
                angle = int(msg.data[0])
                awake = int(msg.data[1]) if len(msg.data) >= 2 else 0
                audio_angle = angle
                audio_awake_flag = awake
                if awake == 1 or angle != last_angle:
                    audio_angle_valid = True
                if audio_angle != last_angle:
                    last_angle = audio_angle
                    self.get_logger().info(f"📍 声源角度更新: {audio_angle}° awake={audio_awake_flag}")

    def odom_callback(self, msg):
        global current_yaw
        _q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([_q.x, _q.y, _q.z, _q.w])
        with odom_lock:
            current_yaw = yaw

    def rotate_to_angle(self):
        """读取当前声源角度，让小车转向到正对声源方向"""
        global audio_angle, current_yaw, audio_angle_valid, audio_awake_flag

        with angle_lock:
            target_angle = audio_angle
            has_angle = audio_angle_valid
            awake_flag = audio_awake_flag

        if target_angle is None or target_angle == -1 or not has_angle:
            self.get_logger().warn("⚠️ 未收到麦克风阵列声源定位包，跳过转向")
            self._send_voice_log("⚠️ 未收到声源角度，跳过转向")
            return False

        target_angle = target_angle % 360
        if target_angle <= 5 or target_angle >= 355:
            self.get_logger().info(f"✅ 声源角度 {target_angle}°，用户在正前方，无需转向")
            self._send_voice_log("✅ 声源在正前方，无需转向")
            return True

        # 计算最短旋转方向。后方角度统一为 180°，减少阵列在正后方
        # 175°/185°之间抖动造成的左右方向切换。
        if 150 <= target_angle <= 210:
            target_angle = 180
        if 0 < target_angle < 180:
            direction = -1.0
            rotate_angle = target_angle
        else:
            direction = 1.0
            rotate_angle = 360 - target_angle

        with odom_lock:
            previous_yaw = current_yaw
        target_radians = math.radians(rotate_angle)
        accumulated = 0.0
        deadline = time.monotonic() + max(6.0, target_radians / 0.35 + 4.0)

        self.get_logger().info(
            f"🔄 转向声源: 原始角度{audio_angle}°，计划旋转"
            f"{rotate_angle:.1f}°，方向{direction}"
        )
        self._send_voice_log(f"🔄 转向声源 {rotate_angle:.0f}°...")

        # 不能只发布一次 cmd_vel：底盘安全看门狗会让长距离转向中途停下，
        # 180° 后方转向因此常常只完成约一半。这里以 12.5Hz 持续刷新，
        # 并用累计里程计角度跨越 ±pi 边界。
        while rclpy.ok():
            with odom_lock:
                current = current_yaw
            delta = math.atan2(
                math.sin(current - previous_yaw),
                math.cos(current - previous_yaw),
            )
            previous_yaw = current
            accumulated += max(0.0, direction * delta)
            remaining = target_radians - accumulated
            if remaining <= math.radians(5.0):
                break
            if time.monotonic() >= deadline:
                self.get_logger().warning(
                    f"⚠️ 声源转向超时，已完成 {math.degrees(accumulated):.0f}°/"
                    f"{rotate_angle:.0f}°"
                )
                self._send_voice_log("⚠️ 声源转向未完全到位，已安全停止")
                for _ in range(3):
                    self.cmd_pub.publish(Twist())
                    time.sleep(0.05)
                return False

            twist = Twist()
            taper = min(1.0, remaining / math.radians(30.0))
            twist.angular.z = direction * max(0.22, 0.6 * taper)
            self.cmd_pub.publish(twist)
            time.sleep(0.08)

        # 停止
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

        self.get_logger().info("✅ 转向完成！")
        self._send_voice_log("✅ 转向完成")
        return True

    # ==================== GUI 联动 ====================
    def voice_trigger_callback(self, msg):
        """处理来自 ui_dashboard.py 语音页面按钮的开关指令"""
        if msg.data == "START_LISTENING":
            self.is_listening = True
        elif msg.data == "STOP_LISTENING":
            self.is_listening = False
        elif msg.data == "TOGGLE_LISTENING":
            self.is_listening = not self.is_listening
        else:
            self.get_logger().warning(f"未知语音面板指令: {msg.data}")
            return
        status = "🟢 麦克风已开启" if self.is_listening else "🔴 麦克风已关闭"
        state_code = "STATUS:ON" if self.is_listening else "STATUS:OFF"
        self.get_logger().info(f"🎤 [GUI联动] {status}")
        self._send_voice_log(status)
        self._send_voice_log(state_code)

    def _send_voice_log(self, text):
        """发送日志到 GUI 语音页面的 QTextBrowser"""
        log_msg = String()
        log_msg.data = text
        self.voice_log_pub.publish(log_msg)

    def reset_voice_angle(self):
        """开始听一句新话前，清空上一句的声源定位状态。"""
        global audio_angle_valid, audio_awake_flag
        with angle_lock:
            audio_angle_valid = False
            audio_awake_flag = 0

    def face_current_speaker(self, label="声源"):
        """每次识别到用户说话后，按最新声源角度转向说话人。"""
        with angle_lock:
            current_angle = audio_angle
            has_angle = audio_angle_valid
        if has_angle:
            self.get_logger().info(f"📍 {label}角度: {current_angle}°")
            self._send_voice_log(f"📍 {label}角度: {current_angle}°")
        return self.rotate_to_angle()

    # ==================== 机械臂控制 ====================
    def send_pose(self, angles):
        """发送机械臂角度"""
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = [float(a) for a in angles]
        self.publisher_.publish(msg)

    def speak_and_play(self, text, publish_dialogue=True):
        """发音模块：百度TTS合成 + 播放"""
        with self.speech_lock:
            clean_text = remove_think_tag(text).strip()
            if not clean_text:
                return

            self.ignore_mic_until = time.time() + 8.0
            self.get_logger().info(f"🎙️ 机器人发音: {clean_text}")
            if publish_dialogue:
                self._send_voice_log(f"🤖 AI: {clean_text}")
                self._publish_care_dialogue("robot", clean_text)

            audio_file = baidu_tts(clean_text)
            if audio_file:
                play_audio_file(audio_file)
            self.ignore_mic_until = time.time() + 1.5

    # ==================== DeepSeek 对话 ====================
    def ask_deepseek_api(self, prompt):
        """聊天模块：挂载天气时间，请求 DeepSeek"""
        global chat_history
        self.get_logger().info("🧠 正在呼叫 DeepSeek 思考...")

        current_time = datetime.datetime.now().strftime("%H点%M分")
        system_prompt = (
            f"你叫小微，是一台家庭服务型机器人，运行在K1 MUSE Pi Pro和ROS2系统上。时间：{current_time}。"
            f"天气：{get_weather()}。"
            "你的职责是陪伴、看护、安全提醒、智能家居协助和简单生活服务。"
            "回答要温柔、可靠、像家里的机器人管家；尽量用简短口语中文，不超过60字。"
            "如果用户提到危险、儿童看护、燃气、烟雾或一氧化碳，要优先提醒安全处理。"
        )
        chat_history[0]["content"] = system_prompt
        chat_history.append({"role": "user", "content": prompt})

        if len(chat_history) > 21:
            chat_history = [chat_history[0]] + chat_history[-20:]

        try:
            response = client_llm.chat.completions.create(
                model="deepseek-chat", messages=chat_history, stream=False
            )
            reply = response.choices[0].message.content
            chat_history.append({"role": "assistant", "content": reply})
            return reply
        except Exception as e:
            chat_history.pop()
            return "指挥官，我的云端大脑暂时掉线了。"

    # ==================== 百度 ASR 识别 ====================
    def baidu_asr(self, wav_data):
        """百度语音识别"""
        try:
            result = self.client_asr.asr(wav_data, 'wav', 16000, {'dev_pid': 1537})
            if result['err_no'] == 0:
                return result['result'][0]
            else:
                self.get_logger().error(f"❌ 语音识别失败: {result.get('err_msg')}")
                return ""
        except Exception as e:
            self.get_logger().error(f"❌ ASR异常: {e}")
            return ""

    def has_human_voice(self, audio, recognizer):
        """在调用云端 ASR 前检查录音中是否存在持续人声。"""
        raw_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
        samples = np.frombuffer(raw_data, dtype=np.int16).astype(np.float32)
        frame_size = 320  # 16 kHz 下每帧 20 ms
        frame_count = len(samples) // frame_size
        if frame_count == 0:
            return False

        frames = samples[:frame_count * frame_size].reshape(frame_count, frame_size)
        rms = np.sqrt(np.mean(frames * frames, axis=1))
        noise_floor = float(np.percentile(rms, 20))
        threshold = max(
            float(self.voice_min_rms),
            noise_floor * self.voice_snr_ratio,
            float(recognizer.energy_threshold) * 0.6,
        )
        voiced = rms >= threshold
        min_frames = max(1, int(math.ceil(self.voice_min_ms / 20.0)))

        longest_run = 0
        current_run = 0
        for is_voiced in voiced:
            if is_voiced:
                current_run += 1
                longest_run = max(longest_run, current_run)
            else:
                current_run = 0

        voiced_frames = int(np.count_nonzero(voiced))
        peak_rms = float(np.max(rms))
        accepted = (
            voiced_frames >= min_frames
            and longest_run >= max(4, min_frames // 2)
            and peak_rms >= threshold * 1.15
        )
        self.get_logger().info(
            "🔎 人声检测: "
            f"峰值={peak_rms:.0f}, 环境={noise_floor:.0f}, 门槛={threshold:.0f}, "
            f"有效={voiced_frames * 20}ms, 连续={longest_run * 20}ms, "
            f"结果={'通过' if accepted else '过滤'}"
        )
        return accepted

    def is_meaningful_text(self, text):
        """过滤云端 ASR 对噪声产生的空白、语气词和极短误识别。"""
        clean_text = re.sub(r'[^\w\u4e00-\u9fff]+', '', str(text), flags=re.UNICODE)
        noise_words = {
            "嗯", "啊", "哦", "噢", "呃", "额", "哎", "唉", "诶", "喂",
            "嗯嗯", "啊啊", "哦哦", "哈哈", "呵呵",
        }
        single_char_commands = {"左", "右", "抓", "松"}
        if not clean_text or clean_text in noise_words:
            return False
        return len(clean_text) >= 2 or clean_text in single_char_commands

    def start_continuous_listener(self):
        """校准一次麦克风，然后在后台持续采集语音。"""
        recognizer = sr.Recognizer()
        recognizer.pause_threshold = 0.7
        recognizer.non_speaking_duration = 0.5
        mic_index = int(os.environ.get("JARVIS_MIC_INDEX", "3"))
        microphone = sr.Microphone(device_index=mic_index, sample_rate=16000)

        print("\n" + "=" * 45)
        with microphone as source:
            self.get_logger().info(f"👂 [仅启动时校准一次] 麦克风设备 index={mic_index}")
            recognizer.adjust_for_ambient_noise(source, duration=1.0)

        recognizer.energy_threshold = max(
            recognizer.energy_threshold,
            float(os.environ.get("JARVIS_TRIGGER_ENERGY", "450")),
        )
        recognizer.dynamic_energy_threshold = False
        self.stop_background_listening = recognizer.listen_in_background(
            microphone,
            self.audio_capture_callback,
            phrase_time_limit=10,
        )
        self.get_logger().info("🎧 后台持续监听已开启，可以随时说话")
        self._send_voice_log("🎧 小微持续监听中，可以随时说话")

    def audio_capture_callback(self, recognizer, audio):
        """后台录音回调只做人声筛选和入队，不执行任何网络请求。"""
        if (
            not self.is_listening
            or self.remote_talk_active.is_set()
            or time.time() < self.ignore_mic_until
        ):
            return

        if not self.has_human_voice(audio, recognizer):
            self.get_logger().info("🔇 环境声已过滤，后台监听未中断")
            return

        try:
            self.audio_queue.put_nowait(audio)
        except queue.Full:
            try:
                self.audio_queue.get_nowait()
                self.audio_queue.task_done()
            except queue.Empty:
                pass
            self.audio_queue.put_nowait(audio)
            self.get_logger().warning("⚠️ 语音队列已满，已保留最新一句话")

    def decode_audio(self, audio):
        """在处理线程中调用百度 ASR，避免阻塞后台录音。"""
        self.get_logger().info("⏳ 百度云端语音解码中...")
        text = self.baidu_asr(audio.get_wav_data())
        if not self.is_meaningful_text(text):
            if text:
                self.get_logger().info(f"🔇 已过滤无意义识别结果: {text}")
            return ""
        return text

    def handle_user_text(self, cmd_text):
        """处理唤醒后的连续对话内容：动作指令优先，其余交给家庭服务聊天。"""
        self.get_logger().info(f"\n👤 用户说: {cmd_text}")
        self._send_voice_log(f"👤 用户: {cmd_text}")
        self._publish_care_dialogue("user", cmd_text)

        clean_text = re.sub(r'[，。！？、,\.!\?\s]+', '', cmd_text)
        sleep_keywords = ["退下", "休眠", "结束聊天", "不用了", "再见", "拜拜"]
        if any(keyword in clean_text for keyword in sleep_keywords):
            self.get_logger().info("💤 用户结束连续对话，回到休眠。")
            self._send_voice_log("💤 已退出连续对话，回到休眠")
            self.speak_and_play("好的，我先待机，有需要再叫我。")
            return "sleep"

        environment_keywords = [
            "室内环境", "屋里环境", "家里环境", "环境如何", "环境怎么样",
            "环境安全吗", "空气质量", "空气安全吗", "室内空气",
            "室内温度", "室内湿度", "温度多少", "湿度多少",
            "二氧化碳", "CO2", "甲醛", "VOC", "PM2.5", "PM10", "颗粒物",
        ]
        if any(keyword.lower() in clean_text.lower() for keyword in environment_keywords):
            self.get_logger().info("🌿 检测到室内环境查询，读取七合一传感器实时数据")
            self._send_voice_log("🌿 正在读取室内环境传感器...")
            reply = self.get_indoor_environment_reply(cmd_text)
            self.speak_and_play(reply)
            return "handled"

        control_keywords = ["回正", "点头", "左", "右", "跳舞", "抓", "松"]

        if any(keyword in clean_text for keyword in control_keywords):
            self.get_logger().info("🦾 检测到【动作指令】，直接控制硬件！")
            self._send_voice_log("🦾 [动作指令] 直接控制硬件...")

            if "回正" in clean_text:
                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                self.speak_and_play("好的，机械臂已回正。")
            elif "点头" in clean_text:
                self.send_pose([0.0, 0.0, 0.0, -1.0, 0.0, 0.0])
                time.sleep(0.5)
                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                self.speak_and_play("我在这里，随时为家里服务。")
            elif "左" in clean_text:
                self.send_pose([-1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                self.speak_and_play("好的，我向左看一下。")
            elif "右" in clean_text:
                self.send_pose([1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                self.speak_and_play("好的，我向右看一下。")
            elif "跳舞" in clean_text:
                self.speak_and_play("收到，给家里来一点气氛。")
                self.send_pose([0.5, 0.0, 0.0, -1.0, 0.0, 0.0])
                time.sleep(0.5)
                self.send_pose([-0.5, 0.0, 0.0, -2.0, 0.0, 0.0])
                time.sleep(0.5)
                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
            elif "抓" in clean_text:
                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, -0.9])
                self.speak_and_play("好的，夹爪已夹紧。")
            elif "松" in clean_text:
                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.9])
                self.speak_and_play("好的，夹爪已松开。")
            return "handled"

        self.get_logger().info("💬 连续聊天模式：呼叫 DeepSeek...")
        self._send_voice_log("💬 [家庭服务聊天] 正在思考...")
        ai_reply = self.ask_deepseek_api(cmd_text)
        self.speak_and_play(ai_reply)
        return "handled"

    # ==================== 核心监听循环 ====================
    def listen_and_act(self):
        """持续录音，说一次“小薇”或常见近音词即可转向并回答。"""
        time.sleep(1)
        try:
            self.start_continuous_listener()
        except Exception as e:
            self.get_logger().error(f"⚠️ 持续监听启动失败: {e}")
            return

        try:
            while rclpy.ok():
                try:
                    if not self.is_listening:
                        time.sleep(0.3)
                        continue

                    try:
                        audio = self.audio_queue.get(timeout=0.5)
                    except queue.Empty:
                        continue

                    try:
                        while (
                            rclpy.ok()
                            and (
                                self.remote_talk_active.is_set()
                                or time.time() < self.ignore_mic_until
                            )
                        ):
                            time.sleep(0.1)

                        if not rclpy.ok() or not self.is_listening:
                            continue

                        cmd_text = self.decode_audio(audio)
                        if not cmd_text:
                            continue

                        wake_command = extract_wake_command(cmd_text)
                        if wake_command is None:
                            self.get_logger().info(
                                f"🔇 未检测到“小薇”或相近唤醒词，忽略本句: {cmd_text}"
                            )
                            self.reset_voice_angle()
                            continue

                        try:
                            self.get_logger().info("✅ 已识别小薇唤醒词")
                            self._send_voice_log("✅ 小薇已唤醒")
                            self.face_current_speaker("唤醒者")
                            if wake_command:
                                self.handle_user_text(wake_command)
                            else:
                                self.speak_and_play("我在，请说。")
                        finally:
                            self.reset_voice_angle()
                    finally:
                        self.audio_queue.task_done()

                except Exception as e:
                    self.get_logger().error(f"⚠️ 循环异常: {e}")
        finally:
            if self.stop_background_listening is not None:
                self.stop_background_listening(wait_for_stop=False)


def main(args=None):
    rclpy.init(args=args)
    node = JarvisCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n🚪 系统已关闭。")
    finally:
        if node.stop_background_listening is not None:
            node.stop_background_listening(wait_for_stop=False)
        if rclpy.ok():
            twist = Twist()
            node.cmd_pub.publish(twist)
        if node.mqtt_client is not None:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("👋 再见！")


if __name__ == '__main__':
    main()
