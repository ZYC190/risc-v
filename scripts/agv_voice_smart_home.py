#!/usr/bin/env python3
# coding=utf-8
"""
==========================================
 AGV 语音全屋智能联动脚本 (8阶段流水线)
==========================================
完整流程:
  阶段1: 车载麦克风收音 → 百度 ASR 语音转文字
  阶段2: DeepSeek K1 极速语义意图识别
  阶段3: 百度 TTS 语音合成 → 扬声器播报回应
  阶段4: MQTT 两路并行发射:
         a) edge/light/cmd → ESP32 多节点 → LED 闪烁 (全屋照明联动)
         b) robot/voice_cmd → agv_master_node → /goal_pose → Nav2 导航到 A 点取水
  阶段5: 等待小车到达 A 点 (ROS2 里程计监听)
  阶段6: 机械臂串口抓取流水线 (举手→下降→夹紧→抬起→归位)
  阶段7: MQTT 导航回原点 (人身边) + 松爪递水
  阶段8: TTS 播报完成提示

运行方式:
  python3 agv_voice_smart_home.py
==========================================
"""

import os
os.environ["PA_ALSA_PLUGHW"] = "1"

import re
import tempfile
import datetime
import threading
import time
import math
import json
import struct
import requests

# ROS2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped

# 音频 & API
from pydub import AudioSegment
from pydub.playback import play
from openai import OpenAI
import speech_recognition as sr
from aip import AipSpeech

# MQTT
import paho.mqtt.client as mqtt

# 串口
import serial

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

# ==========================================
# 🗺️ 战略坐标本
# ==========================================
TARGET_LOCATIONS = {
    "A_point": {
        "x": 1.0771608352661133,
        "y": -0.02679574303328991,
        "qz": 0.1505363651089196,
        "qw": 0.9886044723648554
    },
    "B_point": {   # 原点 / 人的位置
        "x": 0.21498864889144897,
        "y": -0.04589308425784111,
        "qz": -0.9893725548949317,
        "qw": 0.145402708436519
    }
}

# ==========================================
# 🤖 机械臂物理参数 (来自 arm_grabber.py)
# ==========================================
ARM_PORT = '/dev/wheeltec_arm'
ARM_BAUD = 115200

# ==========================================
# 🌐 MQTT 配置
# ==========================================
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_LIGHT_TOPIC = "edge/light/cmd"       # 全屋灯光控制
MQTT_VOICE_CMD_TOPIC = "robot/voice_cmd"   # 导航指令

# 导航超时 (秒)
NAV_TIMEOUT = 120.0

# ==========================================
# 全局变量
# ==========================================
client_llm = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")
BAIDU_TOKEN = None

# 里程计当前位姿
current_odom_x = 0.0
current_odom_y = 0.0
current_odom_qz = 0.0
current_odom_qw = 1.0
odom_lock = threading.Lock()

# 导航到达标志
nav_arrived = False
nav_arrived_lock = threading.Lock()

# /goal_pose publisher (直接发布导航目标，不依赖 agv_master_node)
goal_publisher = None

# 全局机械臂控制器实例 (程序启动时初始化，全程复用)
arm_controller = None


# ==================== 百度 API 工具函数 ====================
def get_baidu_token():
    """获取百度 API 的 Access Token"""
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


def speak(text):
    """一站式 TTS 合成 + 播放"""
    print(f"🎙️ 机器人发声: {text}")
    clean_text = re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL).strip()
    if not clean_text:
        return
    audio_file = baidu_tts(clean_text)
    if audio_file:
        play_audio(audio_file)


def listen_and_recognize(timeout=10, phrase_limit=10):
    """用USB麦克风听一次，返回百度ASR识别文字"""
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

        client_asr = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
        result = client_asr.asr(wav_data, 'wav', 16000, {'dev_pid': 1537})
        if result['err_no'] == 0:
            return result['result'][0]
        else:
            print(f"❌ 语音识别失败: {result.get('err_msg')}")
            return ""
    except sr.WaitTimeoutError:
        print("💤 没有听到声音...")
        return ""
    except Exception as e:
        print(f"⚠️ 麦克风或识别异常: {e}")
        return ""


# ==================== DeepSeek K1 极速意图识别 ====================
def intent_recognition(user_text):
    """
    调用 DeepSeek API 进行 K1 极速语义意图识别
    返回 JSON: {"intent": "...", "confidence": 0.xx}
    支持意图:
      - fetch_water: 取水
      - turn_on_lights: 开灯
      - fetch_water_and_lights: 取水+开灯
      - go_fetch: 去取东西
      - unknown: 无法识别
    """
    print("🧠 DeepSeek K1 极速意图识别中...", end='', flush=True)

    system_prompt = (
        "你是一个极速意图分类器，安装在AGV小车上。"
        "请分析用户语音，仅输出一个 JSON 对象，不要输出任何其他内容。"
        "JSON 格式: {\"intent\": \"<类别>\", \"confidence\": <0到1之间的数字>}\n"
        "意图类别说明:\n"
        "- fetch_water: 用户要求去取水、拿水、打水、接水\n"
        "- turn_on_lights: 用户要求开灯、照明、亮灯\n"
        "- fetch_water_and_lights: 用户同时要求取水和开灯\n"
        "- go_fetch: 用户要求去取东西(非水)\n"
        "- unknown: 无法匹配上述意图\n"
        "示例输入: '去帮我取瓶水，顺便把灯打开' → {\"intent\": \"fetch_water_and_lights\", \"confidence\": 0.98}\n"
        "示例输入: '开灯' → {\"intent\": \"turn_on_lights\", \"confidence\": 0.99}\n"
        "示例输入: '去取水' → {\"intent\": \"fetch_water\", \"confidence\": 0.97}"
    )

    try:
        response = client_llm.chat.completions.create(
            model="deepseek-chat",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_text}
            ],
            stream=False,
            temperature=0.1,   # 极低温度，保证分类一致
            max_tokens=100
        )
        raw = response.choices[0].message.content.strip()
        print(f" [识别完毕] → {raw}")

        # 尝试提取 JSON
        # 处理可能的 markdown 包裹
        json_match = re.search(r'\{[^{}]*\}', raw)
        if json_match:
            result = json.loads(json_match.group())
            return result
        else:
            print(f"⚠️ 意图识别返回非JSON格式: {raw}")
            return {"intent": "unknown", "confidence": 0.0}

    except (json.JSONDecodeError, ValueError) as e:
        print(f"\n❌ 意图解析失败: {e}, 原始: {raw}")
        return {"intent": "unknown", "confidence": 0.0}
    except Exception as e:
        print(f"\n❌ DeepSeek 意图识别失败: {e}")
        return {"intent": "unknown", "confidence": 0.0}


# ==================== MQTT 发布工具 ====================
def mqtt_publish(topic, payload):
    """同步发布一条 MQTT 消息（每次新建连接，用完即断）"""
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "agv_smart_home")
        client.connect(MQTT_BROKER, MQTT_PORT, 10)
        client.loop_start()
        time.sleep(0.1)
        client.publish(topic, payload)
        time.sleep(0.1)
        client.loop_stop()
        client.disconnect()
        print(f"📡 MQTT 已发射 → [{topic}]: {payload}")
        return True
    except Exception as e:
        print(f"❌ MQTT 发射失败 [{topic}]: {e}")
        return False


# ==================== 机械臂串口+双目视觉控制 (整合自 arm_grabber.py) ====================
class ArmController(Node):
    """机械臂控制器 (ROS2节点, 订阅双目相机 /target_point, 逆运动学解算)"""

    def __init__(self):
        super().__init__('arm_controller_node')

        # ==========================================
        # ⚡ 指挥官精测物理尺寸参数 (轴心到轴心)
        # ==========================================
        self.link_a = 0.105           # 大臂轴距: 10.5cm
        self.link_c = 0.100           # 小臂轴距: 10.0cm
        self.link_gripper = 0.150     # J4轴心到两片夹爪闭合中心的净长度: 15.0cm

        # ==========================================
        # 🎯 核心物理标定区 (立体空间外参对齐)
        # ==========================================
        self.measured_cam_z = 0.32
        self.measured_horizontal_y = 0.28   # 28cm 水平距离
        self.camera_offset_y = -0.08  # 底座到相机真实水平距离: 8cm
        self.camera_offset_z = 0.30   # 相机距离底座垂直高度: 30cm

        # 🔍 毫米级极细偏置修正
        self.x_offset = -0.020
        self.y_offset = 0.010
        self.z_offset = -0.025     # 高度 1.5cm 降落修正

        # ==========================================
        # 🎯 多帧平均累积区
        # ==========================================
        self.samples_needed = 5          # 收集几张图的坐标
        self.collected_x = []            # 累积 x 坐标
        self.collected_y = []            # 累积 y 坐标
        self.collected_z = []            # 累积 z 坐标

        # 最近一次计算出的 IK 关节角 (用于抓取)
        self.latest_j1 = 0.0
        self.latest_j2 = 0.6
        self.latest_j3 = -0.5
        self.latest_j4 = -1.07
        self.latest_j5 = 0.0
        self.ik_ready = False            # IK 是否已计算过有效目标

        # ==========================================
        # 🛡️ 抓取完成标志位 (防止重复抓取)
        # ==========================================
        self.has_executed = False

        # ==========================================
        # 1. 串口初始化
        # ==========================================
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(ARM_PORT, ARM_BAUD, timeout=0.5)
            print(f"🔥 成功连接 STM32 串口: {ARM_PORT}")
        except Exception as e:
            print(f"❌ 串口打开失败: {e}")
            return

        print("⏳ 等待串口硬件接收器整备...")
        time.sleep(1.0)

        # 开机回正 (将机械臂移到初始归位位置)
        print("🤖 正在向单片机发射初始化归位密令...")
        self.send_joint_angles(0.0, 1.0, -1.57, -1.57, 0.0, 0.0, mode=2)
        time.sleep(1.5)
        print("✅ 机械臂初始化归位完成！")

        # ==========================================
        # 2. 订阅双目视觉话题 /target_point
        # ==========================================
        self.subscription = self.create_subscription(
            PointStamped,
            '/target_point',
            self.target_callback,
            10)
        print("👁️ 已订阅双目相机话题 /target_point，等待目标坐标...")

    def send_joint_angles(self, rad1, rad2, rad3, rad4, rad5, rad6, mode=1):
        """严格大端序打包发送函数 (与 arm_grabber.py 完全一致)"""
        if not self.serial_port or not self.serial_port.is_open:
            print("⚠️ 串口未就绪，跳过发送")
            return

        j1 = int(rad1 * 1000)
        j2 = int(rad2 * 1000)
        j3 = int(rad3 * 1000)
        j4 = int(rad4 * 1000)
        j5 = int(rad5 * 1000)
        j6 = int(rad6 * 1000)

        data = bytearray(16)
        data[0] = 0xAA

        struct.pack_into('>h', data, 1, j1)
        struct.pack_into('>h', data, 3, j2)
        struct.pack_into('>h', data, 5, j3)
        struct.pack_into('>h', data, 7, j4)
        struct.pack_into('>h', data, 9, j5)
        struct.pack_into('>h', data, 11, j6)

        data[13] = mode

        check_sum = 0
        for i in range(14):
            check_sum ^= data[i]
        data[14] = check_sum
        data[15] = 0xBB

        try:
            self.serial_port.write(data)
            self.serial_port.flush()
            print(f"   🔧 串口发射: J1={rad1:.2f} J2={rad2:.2f} J3={rad3:.2f} J4={rad4:.2f} J5={rad5:.2f} J6={rad6:.2f} mode={mode}")
        except Exception as e:
            print(f"❌ 串口数据发射异常: {e}")

    def target_callback(self, msg):
        """接收双目相机 /target_point 坐标, 累积多帧后计算逆运动学关节角"""
        # 🛡️ 已经完成抓取，忽略后续所有坐标
        if self.has_executed:
            return

        cam_x = msg.point.x
        cam_z = msg.point.z

        # 🎯 收集坐标样本
        self.collected_x.append(cam_x)
        self.collected_y.append(0.0)
        self.collected_z.append(cam_z)

        # ⏳ 样本数不足，继续等待
        if len(self.collected_x) < self.samples_needed:
            print(f"📊 双目坐标收集: {len(self.collected_x)}/{self.samples_needed} 帧 (x={cam_x:.3f}, z={cam_z:.3f})")
            return

        # ✅ 样本充足，计算平均值
        avg_x = sum(self.collected_x) / len(self.collected_x)
        avg_z = sum(self.collected_z) / len(self.collected_z)
        print(f"🎯 双目坐标平均完成: x={avg_x:.3f}, z={avg_z:.3f} （共 {len(self.collected_x)} 帧）")

        # 清空累积区，准备下一轮
        self.collected_x.clear()
        self.collected_y.clear()
        self.collected_z.clear()

        # ==========================================
        # 逆运动学解算 (与 arm_grabber.py 完全一致)
        # ==========================================
        cam_x = avg_x
        cam_z = avg_z

        # 1. 空间几何投影
        cos_pitch = self.measured_horizontal_y / self.measured_cam_z
        horizontal_depth = cam_z * cos_pitch

        # 2. 转换至机械臂底座坐标系
        bottle_target_y = horizontal_depth + self.camera_offset_y + self.y_offset
        bottle_target_x = cam_x + self.x_offset

        # 3. 实时推算瓶子原本的 3D 空间高度
        sin_pitch = math.sqrt(1 - cos_pitch**2)
        vertical_drop = cam_z * sin_pitch
        bottle_target_z = self.camera_offset_z - vertical_drop

        # 4. 🎯 TCP剥离与真实高度沉降算法
        Y_wrist = bottle_target_y - self.link_gripper
        Z_wrist = bottle_target_z + self.z_offset

        # 5. 纯几何余弦定理解算大臂与小臂
        D2 = Y_wrist**2 + Z_wrist**2
        cos_j3_raw = (D2 - self.link_a**2 - self.link_c**2) / (2 * self.link_a * self.link_c)
        cos_j3_raw = max(-1.0, min(1.0, cos_j3_raw))
        j3_angle = math.acos(cos_j3_raw)

        psi = math.atan2(Y_wrist, Z_wrist)
        cos_mu = (self.link_a**2 + D2 - self.link_c**2) / (2 * self.link_a * math.sqrt(D2))
        cos_mu = max(-1.0, min(1.0, cos_mu))
        mu = math.acos(cos_mu)

        # 姿态方向合成 (负数向前低头)
        j1 = math.atan2(bottle_target_x, bottle_target_y) if bottle_target_y != 0 else 0.0

        # 拱起折叠构型
        j2 = -(psi - mu)
        j3 = -j3_angle

        # 🌟 绝对水平锁死公式
        j4 = -1.57 - j2 - j3
        j5 = 0.0

        # 🛡️ 镜像 STM32 control.c 硬限幅安全拦截
        if j1 < -1.57 or j1 > 1.57: j1 = max(-1.57, min(1.57, j1))
        if j2 < -1.57 or j2 > 1.57: j2 = max(-1.57, min(1.57, j2))
        if j3 < -1.57 or j3 > 1.57: j3 = max(-1.57, min(1.57, j3))
        if j4 < -0.45 or j4 > 1.57: j4 = max(-0.45, min(1.57, j4))

        # 保存最新解算结果
        self.latest_j1 = j1
        self.latest_j2 = j2
        self.latest_j3 = j3
        self.latest_j4 = j4
        self.latest_j5 = j5
        self.ik_ready = True

        print(f"🧮 IK解算完成 -> J1:{j1:.3f} J2:{j2:.3f} J3:{j3:.3f} J4:{j4:.3f} J5:{j5:.3f}")

    def grab_water_bottle(self):
        """
        执行完整抓取流水线 (基于双目相机 IK 解算坐标):
        动作1: 手臂高高拱起 → 动作2: 向下沉降 → 动作3: 夹紧 → 动作4: 抬起 → 动作5: 归位
        """
        print("\n" + "=" * 50)
        print("🦾 启动机械臂抓取流水线 (双目视觉引导)...")
        print("=" * 50)

        # 使用 IK 解算的关节角 (如果 IK 未就绪则使用默认安全值)
        if self.ik_ready:
            j1, j2, j3, j4, j5 = self.latest_j1, self.latest_j2, self.latest_j3, self.latest_j4, self.latest_j5
            print(f"   📐 使用双目IK关节角: J1={j1:.3f} J2={j2:.3f} J3={j3:.3f} J4={j4:.3f}")
        else:
            # 回退：使用预设固定姿态 (IK未收到坐标时的安全兜底)
            j1, j2, j3, j4, j5 = 0.0, 0.6, -0.5, -1.07, 0.0
            print(f"   ⚠️ 双目IK未就绪，使用默认预设姿态")

        print("🤖 动作 1/5: 手臂高高拱起，向目标上方平滑逼近...")
        self.send_joint_angles(j1, j2 + 0.06, j3, j4 - 0.06, j5, 1.57, mode=2)
        time.sleep(2.0)

        print("🤖 动作 2/5: 向下沉降，预留充足时间消除重力震动...")
        self.send_joint_angles(j1, j2, j3, j4, j5, 1.57, mode=2)
        time.sleep(3.0)

        print("🤖 动作 3/5: 全身完全静止，合拢夹爪进行完美咬合！")
        self.send_joint_angles(j1, j2, j3, j4, j5, -0.4, mode=2)
        time.sleep(1.2)

        print("🤖 动作 4/5: 成功捕获，垂直向上抬起...")
        self.send_joint_angles(j1, j2 + 0.3, j3, j4 - 0.3, j5, -0.4, mode=2)
        time.sleep(2.0)

        print("🤖 动作 5/5: 安全归位（直立夹持）。")
        self.send_joint_angles(0.0, 0.0, 0.0, 0.0, 0.0, -0.4, mode=2)
        time.sleep(1.0)

        # 标记抓取完成，停止收集双目坐标
        self.has_executed = True
        print("✅ 机械臂抓取流水线完成！水瓶已夹持。")

    def reset_samples(self):
        """清空双目坐标累积区，用于车辆停稳后重新采集稳定坐标"""
        self.collected_x.clear()
        self.collected_y.clear()
        self.collected_z.clear()
        self.ik_ready = False
        self.has_executed = False
        print("🔄 双目坐标累积区已清空，等待车辆完全停稳后重新采集...")

    def release_gripper(self):
        """松爪递水"""
        print("\n🖐️ 正在松开夹爪，递水给主人...")
        self.send_joint_angles(0.0, 0.0, 0.0, 0.0, 0.0, 1.57, mode=2)
        time.sleep(1.0)
        print("✅ 夹爪已张开，水瓶已递出！")

    def close(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("🔌 机械臂串口已关闭")


# ==================== ROS2 里程计监听节点 ====================
class OdomMonitorNode(Node):
    """监听里程计，判断小车是否到达目标点"""

    def __init__(self, target_x, target_y, arrival_threshold=0.08):
        super().__init__('odom_monitor_node')
        self.target_x = target_x
        self.target_y = target_y
        self.arrival_threshold = arrival_threshold  # 到达判定距离(米)
        self.arrived = False

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.get_logger().info(f"📍 里程计监听已就绪，目标: ({target_x:.3f}, {target_y:.3f}) 阈值: {arrival_threshold}m")

    def odom_callback(self, msg):
        global current_odom_x, current_odom_y, current_odom_qz, current_odom_qw, nav_arrived
        with odom_lock:
            current_odom_x = msg.pose.pose.position.x
            current_odom_y = msg.pose.pose.position.y
            current_odom_qz = msg.pose.pose.orientation.z
            current_odom_qw = msg.pose.pose.orientation.w

        # 检查是否到达
        dx = current_odom_x - self.target_x
        dy = current_odom_y - self.target_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.arrival_threshold and not self.arrived:
            self.arrived = True
            with nav_arrived_lock:
                nav_arrived = True
            self.get_logger().info(f"🎯 已到达目标点！距离: {dist:.3f}m")


# ==================== 核心业务流程 ====================

def wait_for_arrival(target_name, timeout=NAV_TIMEOUT):
    """
    等待小车导航到达目标点
    通过 ROS2 里程计监听判断
    """
    global nav_arrived

    target = TARGET_LOCATIONS.get(target_name)
    if not target:
        print(f"❌ 未知目标点: {target_name}")
        return False

    with nav_arrived_lock:
        nav_arrived = False

    # 检查是否已经在目标附近
    with odom_lock:
        dx = current_odom_x - target["x"]
        dy = current_odom_y - target["y"]
    initial_dist = math.sqrt(dx * dx + dy * dy)

    print(f"\n⏳ 等待导航到达 [{target_name}] (超时 {timeout}s)...")
    print(f"   当前位置: ({current_odom_x:.3f}, {current_odom_y:.3f})")
    print(f"   目标位置: ({target['x']:.3f}, {target['y']:.3f})")
    print(f"   初始距离: {initial_dist:.2f}m")

    start_time = time.time()
    last_log_time = start_time

    while time.time() - start_time < timeout:
        with nav_arrived_lock:
            if nav_arrived:
                elapsed = time.time() - start_time
                print(f"✅ 导航完成！耗时 {elapsed:.1f}s")
                return True

        # 每秒打印一次距离
        if time.time() - last_log_time > 2.0:
            with odom_lock:
                cx, cy = current_odom_x, current_odom_y
            dist = math.sqrt((cx - target["x"]) ** 2 + (cy - target["y"]) ** 2)
            elapsed = time.time() - start_time
            print(f"   ⏱ {elapsed:.0f}s | 剩余距离: {dist:.2f}m | 当前位置: ({cx:.3f}, {cy:.3f})")
            last_log_time = time.time()

        time.sleep(0.5)

    print(f"⚠️ 导航超时 ({timeout}s)，小车可能未到达目标")
    # 即使超时也尝试继续，检查当前距离
    with odom_lock:
        cx, cy = current_odom_x, current_odom_y
    dist = math.sqrt((cx - target["x"]) ** 2 + (cy - target["y"]) ** 2)
    if dist < 0.08:
        print(f"   当前距离 {dist:.3f}m < 0.08m，视为已精确到达")
        return True
    print(f"   当前距离 {dist:.3f}m >= 0.08m，未精确到达目标点")
    return False


def navigate_to(target_name):
    """直接发布 PoseStamped 到 /goal_pose (绕过 agv_master_node，无需 MQTT 中转)"""
    global goal_publisher

    if target_name not in TARGET_LOCATIONS:
        print(f"❌ 未知目标点: {target_name}")
        return False

    coords = TARGET_LOCATIONS[target_name]

    # 确定中文名称用于日志
    if target_name == "A_point":
        nav_text = "去A点"
    elif target_name == "B_point":
        nav_text = "回原点"
    else:
        nav_text = f"去{target_name}"

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    goal_pose.pose.position.x = coords["x"]
    goal_pose.pose.position.y = coords["y"]
    goal_pose.pose.orientation.z = coords["qz"]
    goal_pose.pose.orientation.w = coords["qw"]

    goal_publisher.publish(goal_pose)
    print(f"\n🗺️ 导航目标已发布到 /goal_pose → {nav_text} ({target_name})")
    return True


def control_lights(state="ON"):
    """通过 MQTT 控制全屋灯光"""
    print(f"\n💡 全屋灯光联动 → {state}")
    return mqtt_publish(MQTT_LIGHT_TOPIC, state)


# ==================== 主流程 ====================
def main():
    global current_odom_x, current_odom_y, nav_arrived

    print("\n" + "=" * 60)
    print("🚀 AGV 语音全屋智能联动系统 启动中...")
    print("=" * 60)
    print("📋 完整流水线:")
    print("   语音收音 → 意图识别 → TTS回应 → MQTT灯光")
    print("   → 导航取水 → 机械臂抓取 → 导航返回 → 递水")
    print("=" * 60)

    # ============ 初始化 ROS2 ============
    global goal_publisher, arm_controller
    rclpy.init()

    # 创建 /goal_pose publisher（替代 agv_master_node 的功能）
    goal_publisher = rclpy.create_node('agv_goal_publisher_node').create_publisher(
        PoseStamped, '/goal_pose', 10
    )
    print("📡 /goal_pose publisher 已就绪")

    # ============ 机械臂初始化 (程序开始时归位) ============
    print("\n🦾 初始化机械臂...")
    arm_controller = ArmController()
    if arm_controller.serial_port and arm_controller.serial_port.is_open:
        print("✅ 机械臂已就绪，双目相机监听中...")
    else:
        print("⚠️ 机械臂串口未就绪，后续抓取将跳过！")

    # ============ 里程计监听 ============
    odom_node = OdomMonitorNode(
        target_x=TARGET_LOCATIONS["A_point"]["x"],
        target_y=TARGET_LOCATIONS["A_point"]["y"],
        arrival_threshold=0.08
    )

    # ROS2 spin 线程 (同时驱动 odom_node 和 arm_controller 两个节点的订阅)
    def ros_spin():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(odom_node)
        executor.add_node(arm_controller)
        try:
            executor.spin()
        except Exception:
            pass

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    time.sleep(1.5)  # 等待 ROS2 和里程计数据就绪

    print("\n✅ 系统全部就绪！请对麦克风说出您的指令...")
    print("   (例如: '去取瓶水并把灯打开')")

    try:
        # ==================== 阶段1: 麦克风收音 + ASR ====================
        print("\n" + "=" * 55)
        print("🎤 [阶段1] 等待语音指令...")
        print("-" * 55)

        user_text = listen_and_recognize(timeout=10, phrase_limit=10)
        if not user_text:
            print("⏰ 未收到语音指令，系统退出。")
            return

        print(f'\n👤 用户说: {user_text}')

        # ==================== 阶段2: DeepSeek K1 意图识别 ====================
        print(f'\n🧠 [阶段2] DeepSeek K1 极速意图识别...')
        intent_result = intent_recognition(user_text)
        intent = intent_result.get("intent", "unknown")
        confidence = intent_result.get("confidence", 0.0)
        print(f"   📊 意图: {intent} | 置信度: {confidence:.2f}")

        # ==================== 阶段3: TTS 语音回应 ====================
        print(f'\n📢 [阶段3] TTS 语音回应...')

        need_water = intent in ("fetch_water", "fetch_water_and_lights", "go_fetch")
        need_lights = intent in ("turn_on_lights", "fetch_water_and_lights")

        if need_water and need_lights:
            response_text = "收到指令，正在前往目标点取水，已为您开启全屋照明。"
        elif need_water:
            response_text = "收到指令，正在前往目标点取水。"
        elif need_lights:
            response_text = "收到指令，已为您开启全屋照明。"
        else:
            response_text = "收到指令，正在执行。"

        speak(response_text)

        # ==================== 阶段4: MQTT 两路并行发射 ====================
        print(f'\n📡 [阶段4] MQTT 控制指令发射...')

        # 4a: 灯光控制
        if need_lights:
            control_lights("ON")
            # 闪烁效果：多按一次制造 LED 闪烁联动
            time.sleep(0.5)
            control_lights("BLINK")
            time.sleep(0.3)
            control_lights("ON")

        # 4b: 导航到 A 点取水
        if need_water:
            navigate_to("A_point")

        # 如果只是开灯不需要导航，直接结束
        if not need_water:
            print("\n✅ 任务完成！(仅灯光控制)")
            return

        # ==================== 阶段5: 等待到达 A 点 ====================
        print(f'\n⏳ [阶段5] 等待小车导航到达 A 点...')

        # 更新 OdomMonitor 目标为 A 点
        with nav_arrived_lock:
            nav_arrived = False
        odom_node.target_x = TARGET_LOCATIONS["A_point"]["x"]
        odom_node.target_y = TARGET_LOCATIONS["A_point"]["y"]
        odom_node.arrived = False

        arrived = wait_for_arrival("A_point", timeout=NAV_TIMEOUT)
        if not arrived:
            print("⚠️ 未能在超时内到达 A 点，继续尝试抓取...")

        # ============ 🔒 锁定当前位置，让Nav2停止微调 ============
        print("\n🔒 正在锁定停车位置，防止车辆微调...")
        with odom_lock:
            lock_x = current_odom_x
            lock_y = current_odom_y
            lock_qz = current_odom_qz
            lock_qw = current_odom_qw
        print(f"   锁定到位姿: x={lock_x:.3f}, y={lock_y:.3f}, qz={lock_qz:.4f}, qw={lock_qw:.4f}")

        # 重复发布当前位姿作为 goal，告诉 Nav2 "你已经到了，别再动了"
        for _ in range(5):
            lock_pose = PoseStamped()
            lock_pose.header.frame_id = 'map'
            lock_pose.header.stamp = rclpy.clock.Clock().now().to_msg()
            lock_pose.pose.position.x = lock_x
            lock_pose.pose.position.y = lock_y
            lock_pose.pose.orientation.z = lock_qz
            lock_pose.pose.orientation.w = lock_qw
            goal_publisher.publish(lock_pose)
            time.sleep(0.05)
        print("🔒 位置锁定完成，车辆已停止微调。")

        # ============ 🛑 停车稳定期 ============
        print("\n🛑 等待车辆完全静止（预留充足稳定时间）...")
        time.sleep(5.0)
        print("✅ 车辆已完全静止。")

        # ============ 🔄 清空并重新采集双目坐标 ============
        if arm_controller.serial_port and arm_controller.serial_port.is_open:
            arm_controller.reset_samples()
            print("⏳ 等待双目相机采集稳定坐标（共需 5 帧）...")

            # 轮询等待 IK 解算就绪
            ik_wait_start = time.time()
            ik_timeout = 15.0
            while not arm_controller.ik_ready:
                if time.time() - ik_wait_start > ik_timeout:
                    print("⚠️ 双目相机 IK 解算超时，将使用默认预设姿态抓取")
                    break
                time.sleep(0.3)
            if arm_controller.ik_ready:
                print(f"✅ 双目坐标采集完成，已获得稳定 IK 关节角")

        # ==================== 阶段6: 机械臂抓取 ====================
        print(f'\n🦾 [阶段6] 机械臂抓取流水线...')

        if arm_controller.serial_port and arm_controller.serial_port.is_open:
            arm_controller.grab_water_bottle()
        else:
            print("❌ 机械臂串口未就绪，跳过抓取！")

        # ==================== 阶段7: 导航回原点 + 松爪递水 ====================
        print(f'\n🏠 [阶段7] 导航回原点 (人身边)...')

        # 更新 OdomMonitor 目标为 B 点 (原点)
        with nav_arrived_lock:
            nav_arrived = False
        odom_node.target_x = TARGET_LOCATIONS["B_point"]["x"]
        odom_node.target_y = TARGET_LOCATIONS["B_point"]["y"]
        odom_node.arrived = False

        navigate_to("B_point")

        arrived_back = wait_for_arrival("B_point", timeout=NAV_TIMEOUT)
        if not arrived_back:
            print("⚠️ 未能在超时内回到原点，继续递水流程...")

        # 停车稳定
        print("🛑 到达原点，停车稳定中...")
        time.sleep(2.0)

        # 松爪递水 (复用启动时初始化的机械臂实例)
        if arm_controller.serial_port and arm_controller.serial_port.is_open:
            arm_controller.release_gripper()
        else:
            print("❌ 机械臂串口未就绪，跳过松爪！")

        # ==================== 阶段8: TTS 完成播报 ====================
        print(f'\n📢 [阶段8] 完成播报...')
        speak("已取水返回，请取走您的水瓶。")

        print("\n" + "=" * 60)
        print("🎉 全流程完成！")
        print("=" * 60)

    except KeyboardInterrupt:
        print('\n🚪 收到强制中断信号，退出系统。')
    except Exception as e:
        print(f"\n❌ 运行异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if arm_controller.serial_port and arm_controller.serial_port.is_open:
            arm_controller.close()
        odom_node.destroy_node()
        rclpy.shutdown()
        print("👋 再见！")


if __name__ == '__main__':
    main()