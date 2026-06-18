#!/usr/bin/env python3
"""
融合版 jarvis_node：声源定位 + 唤醒词 + 机械臂控制 + DeepSeek聊天
- 默认麦克风开启，持续监听唤醒词 "小薇小薇"
- 叫"小薇小薇" → 从麦克风阵列获取声源角度 → 小车转向声源 → 进入聊天模式
- 没说"小薇小薇"时完全不响应聊天
- GUI 语音页面按钮可开启/关闭麦克风
- 机械臂动作指令始终可用（唤醒后执行）
"""
import os
os.environ["PA_ALSA_PLUGHW"] = "1"

import re
import tempfile
import datetime
import threading
import time
import math
import requests

import rclpy
from rclpy.node import Node
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
# ⚠️ 战车核心通信密钥配置
# ==========================================
DEEPSEEK_API_KEY = "YOUR_DEEPSEEK_API_KEY"

BAIDU_APP_ID = "YOUR_BAIDU_APP_ID"
BAIDU_API_KEY = "YOUR_BAIDU_API_KEY"
BAIDU_SECRET_KEY = "YOUR_BAIDU_SECRET_KEY"

GAODE_API_KEY = "YOUR_GAODE_API_KEY"
CITY_CODE = "500000"
# ==========================================

# 初始化全局云端客户端
client_llm = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")
BAIDU_TOKEN = None
chat_history = [{"role": "system", "content": "初始化人设"}]

# 声源角度（由ROS2回调更新）
audio_angle = 0
last_angle = -999
angle_lock = threading.Lock()

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
            play(fixed_audio)
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

        # ------ GUI 联动 ------
        self.voice_log_pub = self.create_publisher(String, '/voice_log', 10)
        self.voice_trigger_sub = self.create_subscription(
            String, '/voice_trigger', self.voice_trigger_callback, 10)

        # ------ 麦克风默认开启 ------
        self.is_listening = True

        self.get_logger().info("🚀 融合版 Jarvis 启动！声源定位 + 唤醒词 + 机械臂 + 聊天")
        self.get_logger().info("🔗 GUI 语音面板联动已就绪 (/voice_log + /voice_trigger)")
        self.get_logger().info("🎤 麦克风默认开启，等待唤醒词 '小薇小薇'...")

        # 启动后台监听线程
        self.listen_thread = threading.Thread(target=self.listen_and_act)
        self.listen_thread.daemon = True
        self.listen_thread.start()

        # 等待线程启动后通知 GUI 初始状态
        time.sleep(0.3)
        self._send_voice_log("STATUS:ON")

    # ==================== 声源定位回调 ====================
    def angle_callback(self, msg):
        global audio_angle, last_angle
        if msg.data and len(msg.data) >= 1:
            with angle_lock:
                audio_angle = msg.data[0]
                if audio_angle != last_angle:
                    last_angle = audio_angle

    def odom_callback(self, msg):
        global current_yaw
        _q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([_q.x, _q.y, _q.z, _q.w])
        with odom_lock:
            current_yaw = yaw

    def rotate_to_angle(self):
        """读取当前声源角度，让小车转向到正对声源方向"""
        global audio_angle, current_yaw

        with angle_lock:
            target_angle = audio_angle

        if target_angle is None or target_angle == -1:
            self.get_logger().warn("⚠️ 未获取到有效声源角度，跳过旋转")
            return False

        with odom_lock:
            odom_start_raw = math.degrees(current_yaw) + 180

        # 计算旋转方向
        if 0 < target_angle < 180:
            direction = 1.0
            rotate_angle = target_angle
        else:
            direction = -1.0
            rotate_angle = 360 - target_angle

        # 目标里程计角度
        if direction < 0:
            odom_target = odom_start_raw - rotate_angle
            if odom_target < 0:
                odom_target += 360
        else:
            odom_target = odom_start_raw + rotate_angle
            if odom_target > 360:
                odom_target -= 360

        self.get_logger().info(
            f"🔄 转向声源: 角度{rotate_angle:.1f}° 方向{direction} | "
            f"当前{odom_start_raw:.1f}° → 目标{odom_target:.1f}°"
        )
        self._send_voice_log(f"� 转向声源 {rotate_angle:.0f}°...")

        twist = Twist()
        twist.angular.z = 0.6 * direction
        self.cmd_pub.publish(twist)

        # 等待旋转到位（误差<6°）
        while rclpy.ok():
            with odom_lock:
                current = math.degrees(current_yaw) + 180
            error = current - odom_target
            if abs(error) < 6:
                break
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
        if msg.data == "TOGGLE_LISTENING":
            self.is_listening = not self.is_listening
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

    # ==================== 机械臂控制 ====================
    def send_pose(self, angles):
        """发送机械臂角度"""
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = [float(a) for a in angles]
        self.publisher_.publish(msg)

    def speak_and_play(self, text):
        """发音模块：百度TTS合成 + 播放"""
        clean_text = remove_think_tag(text).strip()
        if not clean_text:
            return

        self.get_logger().info(f"🎙️ 机器人发音: {clean_text}")
        self._send_voice_log(f"🤖 AI: {clean_text}")

        audio_file = baidu_tts(clean_text)
        if audio_file:
            play_audio_file(audio_file)

    # ==================== DeepSeek 对话 ====================
    def ask_deepseek_api(self, prompt):
        """聊天模块：挂载天气时间，请求 DeepSeek"""
        global chat_history
        self.get_logger().info("🧠 正在呼叫 DeepSeek 思考...")

        current_time = datetime.datetime.now().strftime("%H点%M分")
        system_prompt = (
            f"你是装在AGV小车上的智能语音助手。时间：{current_time}。"
            f"天气：{get_weather()}。"
            f"用简短、口语化、幽默的中文回答，不超过50字。"
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

    def listen_once(self, timeout=5, phrase_limit=3):
        """用USB麦克风听一次，返回识别文字"""
        recognizer = sr.Recognizer()
        recognizer.pause_threshold = 0.8
        try:
            with sr.Microphone(device_index=0, sample_rate=16000) as source:
                print("\n" + "=" * 45)
                self.get_logger().info("👂 [环境静音校准...] ")
                recognizer.adjust_for_ambient_noise(source, duration=1.0)
                self.get_logger().info("🎤 [请说话...] ")
                audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_limit)
            self.get_logger().info("⏳ 百度云端语音解码中...")
            wav_data = audio.get_wav_data()
            text = self.baidu_asr(wav_data)
            return text if text else ""
        except sr.WaitTimeoutError:
            return ""
        except Exception as e:
            self.get_logger().error(f"⚠️ 麦克风或识别异常: {e}")
            return ""

    # ==================== 核心监听循环 ====================
    def listen_and_act(self):
        """核心监听与分发循环：唤醒词检测 → 声源转向 → 指令/聊天"""
        time.sleep(1)

        while rclpy.ok():
            try:
                # 如果麦克风被关闭（通过GUI），休眠等待重新开启
                if not self.is_listening:
                    time.sleep(1.0)
                    continue

                # ========== 阶段1: 等待唤醒词 ==========
                self.get_logger().info("💤 休眠中，说 '小薇小薇' 来唤醒我...")
                self._send_voice_log("💤 等待唤醒词 '小薇小薇'...")

                woke_up = False
                while not woke_up and rclpy.ok():
                    # 检查麦克风是否在循环中被关闭
                    if not self.is_listening:
                        break

                    text = self.listen_once(timeout=5, phrase_limit=3)
                    if not text:
                        continue

                    # 清洗文本：去掉标点符号和空白
                    clean_text = re.sub(r'[，。！？、,\.!\?\s]+', '', text)
                    # 统一替换"小微"→"小薇"
                    clean_text = clean_text.replace("小微", "小薇")
                    self.get_logger().info(f"  � 听到: {text}  →  清洗后: {clean_text}")
                    self._send_voice_log(f"👂 听到: {text}")

                    if "小薇" in clean_text:
                        xw_count = clean_text.count("小薇")
                        if xw_count >= 2 or "小薇小薇" in clean_text:
                            woke_up = True
                            self.get_logger().info("\n🎯 检测到唤醒词【小薇小薇】！")
                            self._send_voice_log("🎯 唤醒词检测到！")

                if not rclpy.ok() or not self.is_listening:
                    continue

                # ========== 阶段2: 声源定位转向 ==========
                with angle_lock:
                    current_angle = audio_angle
                self.get_logger().info(f"  声源角度: {current_angle}°")
                self._send_voice_log(f"📍 声源角度: {current_angle}°")

                # 执行转向
                self.rotate_to_angle()

                # 语音反馈
                self.speak_and_play("我在呢，请说")

                # ========== 阶段3: 指令模式 ==========
                self.get_logger().info("\n🎤 请说出指令（5秒超时）...")
                self._send_voice_log("🎤 请说出指令...")

                cmd_text = self.listen_once(timeout=5, phrase_limit=8)

                if not cmd_text:
                    self.get_logger().info("⏰ 未收到指令，回到休眠模式")
                    self._send_voice_log("⏰ 超时，回到休眠")
                    continue

                self.get_logger().info(f"\n👤 你说: {cmd_text}")
                self._send_voice_log(f"👤 用户: {cmd_text}")

                # ========== 阶段4: 分发 - 动作指令 or 聊天 ==========
                control_keywords = ["回正", "点头", "左", "右", "跳舞", "抓", "松"]

                if any(keyword in cmd_text for keyword in control_keywords):
                    self.get_logger().info("🦾 检测到【动作指令】，直接控制硬件！")
                    self._send_voice_log("🦾 [动作指令] 直接控制硬件...")

                    if "回正" in cmd_text:
                        self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                        self.speak_and_play("好的，已回正。")
                    elif "点头" in cmd_text:
                        self.send_pose([0.0, 0.0, 0.0, -1.0, 0.0, 0.0])
                        time.sleep(0.5)
                        self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                        self.speak_and_play("老板好！")
                    elif "左" in cmd_text:
                        self.send_pose([-1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                        self.speak_and_play("正在向左看。")
                    elif "右" in cmd_text:
                        self.send_pose([1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                        self.speak_and_play("正在向右看。")
                    elif "跳舞" in cmd_text:
                        self.speak_and_play("音乐起，看我摇摆！")
                        self.send_pose([0.5, 0.0, 0.0, -1.0, 0.0, 0.0])
                        time.sleep(0.5)
                        self.send_pose([-0.5, 0.0, 0.0, -2.0, 0.0, 0.0])
                        time.sleep(0.5)
                        self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                    elif "抓" in cmd_text:
                        self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, -0.9])
                        self.speak_and_play("爪子已夹紧。")
                    elif "松" in cmd_text:
                        self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.9])
                        self.speak_and_play("爪子已张开。")

                else:
                    self.get_logger().info("💬 检测为【日常聊天】，呼叫 DeepSeek...")
                    self._send_voice_log("💬 [闲聊模式] 呼叫 DeepSeek...")
                    ai_reply = self.ask_deepseek_api(cmd_text)
                    self.speak_and_play(ai_reply)

            except Exception as e:
                self.get_logger().error(f"⚠️ 循环异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JarvisCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🚪 系统已关闭。")
    finally:
        # 停止小车
        twist = Twist()
        node.cmd_pub.publish(twist)
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
        print("👋 再见！")


if __name__ == '__main__':
    main()