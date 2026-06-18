#!/usr/bin/env python3
"""
融合版：声源定位 + 唤醒词 + 百度API对话
- USB麦克风 (speech_recognition) 持续监听唤醒词 "小薇小薇"
- 叫"小薇小薇" → 从麦克风阵列获取声源角度 → 小车转向声源
- 转向完成后 → 进入指令模式 → 百度ASR识别 → DeepSeek聊天 / 百度TTS回复
- 没说"小薇小薇"时完全不回应
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

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

# 音频 & API
from pydub import AudioSegment
from pydub.playback import play
from openai import OpenAI
import speech_recognition as sr
from aip import AipSpeech

# ==========================================
# ⚠️ API 密钥配置
# ==========================================
DEEPSEEK_API_KEY = "YOUR_DEEPSEEK_API_KEY"
BAIDU_APP_ID = "YOUR_BAIDU_APP_ID"
BAIDU_API_KEY = "YOUR_BAIDU_API_KEY"
BAIDU_SECRET_KEY = "YOUR_BAIDU_SECRET_KEY"
GAODE_API_KEY = "YOUR_GAODE_API_KEY"
CITY_CODE = "500000"
# ==========================================

# 全局变量
client_llm = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")
BAIDU_TOKEN = None
chat_history = [{"role": "system", "content": "初始化人设"}]

# 声源角度（由ROS2回调更新）
audio_angle = 0
last_angle = -999  # 上次的角度值，用于检测变化
angle_lock = threading.Lock()

# 里程计当前朝向
current_yaw = 0.0
odom_lock = threading.Lock()


# ==================== 百度API工具函数 ====================
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


def play_audio(audio_file):
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


def remove_think_tag(text):
    return re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL)


def get_weather():
    if not GAODE_API_KEY:
        return "暂未配置气象密钥。"
    url = f"https://restapi.amap.com/v3/weather/weatherInfo?city={CITY_CODE}&key={GAODE_API_KEY}"
    try:
        res = requests.get(url, timeout=3).json()
        if res.get("status") == "1" and len(res.get("lives", [])) > 0:
            w = res["lives"][0]
            return f"{w['city']}天气{w['weather']}，气温{w['temperature']}度，{w['winddirection']}风{w['windpower']}级。"
    except Exception:
        pass
    return "气象卫星连接失败。"


def ask_deepseek_api(prompt):
    """呼叫 DeepSeek API 获取回复"""
    global chat_history
    print("🧠 正在呼叫 DeepSeek...", end='', flush=True)

    current_time = datetime.datetime.now().strftime("%H点%M分")
    system_prompt = (
        f"你是装在AGV小车上的智能语音助手。现在时间是：{current_time}。"
        f"当前天气：{get_weather()}。"
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
        print(" [完毕]")
        ai_reply = response.choices[0].message.content
        chat_history.append({"role": "assistant", "content": ai_reply})
        return ai_reply
    except Exception as e:
        print(f"\n❌ 云端呼叫失败: {e}")
        chat_history.pop()
        return "抱歉，我的大脑连接中断了。"


def baidu_asr(wav_data):
    """百度语音识别"""
    client_asr = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
    try:
        result = client_asr.asr(wav_data, 'wav', 16000, {'dev_pid': 1537})
        if result['err_no'] == 0:
            return result['result'][0]
        else:
            print(f"❌ 语音识别失败: {result.get('err_msg')}")
            return ""
    except Exception as e:
        print(f"❌ ASR异常: {e}")
        return ""


def listen_and_recognize(timeout=10, phrase_limit=10):
    """用USB麦克风听一次，返回识别文字（百度ASR）—— 对齐 main_api_tts.py 风格"""
    recognizer = sr.Recognizer()
    recognizer.pause_threshold = 0.8
    try:
        with sr.Microphone(device_index=0, sample_rate=16000) as source:
            print("\n" + "=" * 45)
            print("👂 [环境静音校准...] ")
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
            print("🟢 [滴！请说话！] 🎤")
            audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_limit)
        print("⏳ 正在发往云端识别你的声音...")
        wav_data = audio.get_wav_data()
        text = baidu_asr(wav_data)
        return text if text else ""
    except sr.WaitTimeoutError:
        print("💤 没有听到声音...")
        return ""
    except Exception as e:
        print(f"⚠️ 麦克风或识别异常: {e}")
        return ""


# ==================== ROS2 声源定位 + 旋转控制节点 ====================
class AngleRotateNode(Node):
    def __init__(self):
        super().__init__('angle_rotate_node')

        # 订阅声源角度话题
        self.angle_sub = self.create_subscription(
            Int32MultiArray, 'angle_topic', self.angle_callback, 10
        )
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        # 发布速度指令
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 20)

        self.get_logger().info("🔊 声源定位+旋转控制节点已就绪")

    def angle_callback(self, msg):
        global audio_angle, last_angle
        if msg.data and len(msg.data) >= 1:
            with angle_lock:
                audio_angle = msg.data[0]
                # 只要接收到新角度就更新（不再依赖 data[1]，因为 C++ 端发布后立即清零）
                if audio_angle != last_angle:
                    last_angle = audio_angle

    def odom_callback(self, msg):
        global current_yaw
        _q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([_q.x, _q.y, _q.z, _q.w])
        with odom_lock:
            current_yaw = yaw

    def rotate_to_angle(self):
        """
        读取当前声源角度，让小车转向到正对声源方向
        返回 True 表示旋转完成，False 表示无有效角度
        """
        global audio_angle, current_yaw

        with angle_lock:
            target_angle = audio_angle

        # 0 度是正前方，是有效角度。只有未获取到时才跳过。
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
            f"🔄 转向声源: 旋转{rotate_angle:.1f}° 方向{direction} | "
            f"当前{odom_start_raw:.1f}° → 目标{odom_target:.1f}°"
        )

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
        return True


# ==================== 主流程 ====================
def main():
    global last_angle

    print("\n" + "=" * 55)
    print("🚀 声源定位融合版语音交互引擎 启动中...")
    print("🔊 唤醒词: 小薇小薇")
    print("📡 正在初始化 ROS2 声源定位 + 百度语音API...")

    # 初始化 ROS2
    rclpy.init()
    angle_node = AngleRotateNode()

    # ROS2 spin线程
    def ros_spin():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(angle_node)
        try:
            executor.spin()
        except Exception:
            pass

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    time.sleep(1.0)  # 等待ROS2就绪

    print("✅ 系统全部就绪！等待唤醒词 '小薇小薇'...")

    try:
        while rclpy.ok():
            # ========== 阶段1: 等待唤醒词 ==========
            print("\n" + "=" * 55)
            print("💤 休眠中，说 '小薇小薇' 来唤醒我...")
            print("-" * 55)

            # 持续监听直到听到"小薇小薇"
            woke_up = False

            while not woke_up and rclpy.ok():
                # 复用统一的 listen_and_recognize（含静音校准和百度ASR）
                text = listen_and_recognize(timeout=5, phrase_limit=3)
                if not text:
                    continue

                # 清洗文本：去掉标点符号和空白，用于唤醒词匹配
                clean_text = re.sub(r'[，。！？、,\.!\?\s]+', '', text)
                # 统一替换"小微"→"小薇"（百度ASR可能识别成同音字）
                clean_text = clean_text.replace("小微", "小薇")
                print(f"  👂 听到: {text}  →  清洗后: {clean_text}")

                # 检查唤醒词：包含"小薇"且出现至少2次
                if "小薇" in clean_text:
                    xw_count = clean_text.count("小薇")
                    if xw_count >= 2 or "小薇小薇" in clean_text:
                        woke_up = True
                        print("\n🎯 检测到唤醒词【小薇小薇】！")

            if not rclpy.ok():
                break

            # ========== 阶段2: 声源定位转向 ==========
            with angle_lock:
                current_angle = audio_angle
            print(f"  声源角度: {current_angle}°")

            # 执行转向
            rotated = angle_node.rotate_to_angle()

            say_text = "我在呢，请说"

            # 语音反馈
            audio_file = baidu_tts(say_text)
            if audio_file:
                play_audio(audio_file)

            # ========== 阶段3: 指令模式 ==========
            print("\n🎤 请说出指令（5秒超时）...")
            print("-" * 55)

            cmd_text = listen_and_recognize(timeout=5, phrase_limit=8)

            if not cmd_text:
                print("⏰ 未收到指令，回到休眠模式")
                continue

            print(f'\n👤 你说: {cmd_text}')

            # 调用 DeepSeek 获取回复
            ai_reply = ask_deepseek_api(cmd_text)
            print(f'🤖 AI回复: {ai_reply}\n')

            # 播放回复
            clean_reply = remove_think_tag(ai_reply).strip()
            if clean_reply:
                print('🎙️ 正在语音合成...')
                output_audio = baidu_tts(clean_reply)
                if output_audio:
                    print('🔊 播放中...')
                    play_audio(output_audio)
                else:
                    print('❌ 语音合成失败')
            else:
                print('❌ 回复为空，跳过语音合成')

    except KeyboardInterrupt:
        print('\n🚪 收到退出信号，系统关闭。')
    finally:
        # 停止小车
        twist = Twist()
        angle_node.cmd_pub.publish(twist)
        time.sleep(0.1)
        angle_node.destroy_node()
        rclpy.shutdown()
        print("👋 再见！")


if __name__ == '__main__':
    main()