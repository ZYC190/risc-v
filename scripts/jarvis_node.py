#!/usr/bin/env python3
import os
os.environ["PA_ALSA_PLUGHW"] = "1"
import re
import tempfile
import datetime
import threading
import time
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pydub import AudioSegment
from pydub.playback import play
from openai import OpenAI
import speech_recognition as sr
from aip import AipSpeech  

# ==========================================
# ⚠️ 战车核心通信密钥配置
# ==========================================
DEEPSEEK_API_KEY = "sk-a4d0a0fa3c304a85b4b8dd6a4eb1ae48"

BAIDU_APP_ID = "121955325"  
BAIDU_API_KEY = "KfFHSRiT6DX0cXRyvCwRehEg"
BAIDU_SECRET_KEY = "25HwPUFPTAZhqkxYDLCB8FBaFK8s8gzG"

GAODE_API_KEY = "75ea666272638e5ece2e33c115db66da"  
CITY_CODE = "500000"  
# ==========================================

# 初始化全局云端客户端
client_llm = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")
BAIDU_TOKEN = None
chat_history = [{"role": "system", "content": "初始化人设"}]

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


# ==========================================
# 🤖 核心 ROS 2 节点类
# ==========================================
class JarvisCommander(Node):
    def __init__(self):
        super().__init__('jarvis_commander_node')
        
        # 1. 初始化机械臂控制神经
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 2. 初始化百度语音识别大脑
        self.client_asr = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
        
        self.get_logger().info("🚀 终极融合装甲启动！具备聊天与机械臂控制双重能力！")
        
        # 3. 启动后台独立监听线程 (防止阻塞 ROS2)
        self.listen_thread = threading.Thread(target=self.listen_and_act)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def send_pose(self, angles):
        """发送机械臂角度（带防弹装甲转换）"""
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = [float(a) for a in angles]
        self.publisher_.publish(msg)

    def speak_and_play(self, text):
        """发音模块：合成语音并用 Python 播放器播放（已修复版）"""
        clean_text = remove_think_tag(text).strip()
        if not clean_text: return
        
        self.get_logger().info(f"🎙️ 机器人发音: {clean_text}")
        token = get_baidu_token()
        if not token: return
            
        url = "https://tsn.baidu.com/text2audio"
        payload = {'tex': clean_text, 'tok': token, 'cuid': 'agv_car_001', 'ctp': 1, 'lan': 'zh', 'spd': 5, 'pit': 5, 'vol': 5, 'per': 4, 'aue': 6}
        try:
            res = requests.post(url, data=payload)
            if res.headers.get('Content-Type') == 'audio/wav':
                temp_wav = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
                temp_wav.write(res.content)
                temp_wav.close()
                
                # 放大音量并强制重采样为 48000Hz
                audio = AudioSegment.from_file(temp_wav.name, parameters=["-loglevel", "quiet"])
                fixed_audio = (audio + 10).set_frame_rate(48000)
                
                # 🔥 用 Python 原生播放器，彻底抛弃 aplay！
                play(fixed_audio)
        except Exception as e:
            self.get_logger().error(f"❌ 语音播放异常: {e}")

    def ask_deepseek_api(self, prompt):
        """聊天模块：挂载天气时间，请求 DeepSeek"""
        global chat_history
        self.get_logger().info("🧠 正在呼叫 DeepSeek 思考...")
        
        current_time = datetime.datetime.now().strftime("%H点%M分")
        system_prompt = f"你是装在AGV小车上的智能语音助手。时间：{current_time}。天气：{get_weather()}。用简短、幽默的中文回答，不超过50字。"
        chat_history[0]["content"] = system_prompt
        chat_history.append({"role": "user", "content": prompt})
        
        if len(chat_history) > 21: 
            chat_history = [chat_history[0]] + chat_history[-20:]

        try:
            response = client_llm.chat.completions.create(model="deepseek-chat", messages=chat_history, stream=False)
            reply = response.choices[0].message.content
            chat_history.append({"role": "assistant", "content": reply})
            return reply
        except Exception as e:
            chat_history.pop() 
            return "指挥官，我的云端大脑暂时掉线了。"

    def listen_and_act(self):
        """核心监听与分发循环"""
        recognizer = sr.Recognizer()
        recognizer.pause_threshold = 0.8  
        time.sleep(1)
        
        while rclpy.ok():
            try:
                # 录音
                with sr.Microphone() as source:
                    print("\n" + "="*45)
                    self.get_logger().info("👂 [加载超灵敏声学雷达...] ")
                    
                    # 🔪 1. 注释掉（关掉）自动校准，不让它自作聪明拉高门槛！
                    # recognizer.adjust_for_ambient_noise(source, duration=1)
                    
                    # 💉 2. 强行注入超低能量阈值！(只要有一点声音就抓取)
                    recognizer.energy_threshold = 150
                    # 🔒 3. 锁死阈值
                    recognizer.dynamic_energy_threshold = False
                    
                    self.get_logger().info("🟢 [请大声说话！] 🎤")
                    # 等待声音
                    audio = recognizer.listen(source, timeout=10, phrase_time_limit=10)
                self.get_logger().info("⏳ 百度云端语音解码中...")
                wav_data = audio.get_wav_data(convert_rate=16000)
                result = self.client_asr.asr(wav_data, 'wav', 16000, {'dev_pid': 1537})
                
                if result['err_no'] == 0:
                    cmd = result['result'][0]
                    self.get_logger().info(f"👤 你说: 【{cmd}】")

                    # ========================================
                    # 🚦 神经枢纽分发：是控制指令还是聊天？
                    # ========================================
                    control_keywords = ["回正", "点头", "左", "右", "跳舞", "抓", "松"]
                    
                    if any(keyword in cmd for keyword in control_keywords):
                        self.get_logger().info("🦾 检测到【动作指令】，拦截聊天系统，直接接管硬件！")
                        if "回正" in cmd:
                            self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            self.speak_and_play("好的，已回正。")
                        elif "点头" in cmd:
                            self.send_pose([0.0, 0.0, 0.0, -1.0, 0.0, 0.0])
                            time.sleep(0.5)
                            self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            self.speak_and_play("老板好！")
                        elif "左" in cmd:
                            self.send_pose([-1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            self.speak_and_play("正在向左看。")
                        elif "右" in cmd:
                            self.send_pose([1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            self.speak_and_play("正在向右看。")
                        elif "跳舞" in cmd:
                            self.speak_and_play("音乐起，看我摇摆！")
                            self.send_pose([0.5, 0.0, 0.0, -1.0, 0.0, 0.0])
                            time.sleep(0.5)
                            self.send_pose([-0.5, 0.0, 0.0, -2.0, 0.0, 0.0])
                            time.sleep(0.5)
                            self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                        elif "抓" in cmd:
                            self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, -0.9]) 
                            self.speak_and_play("爪子已夹紧。")
                        elif "松" in cmd:
                            self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.9]) 
                            self.speak_and_play("爪子已张开。")
                    
                    else:
                        self.get_logger().info("💬 检测为【日常聊天】，呼叫 DeepSeek...")
                        ai_reply = self.ask_deepseek_api(cmd)
                        self.speak_and_play(ai_reply)

            except sr.WaitTimeoutError:
                pass
            except Exception as e:
                self.get_logger().error(f"⚠️ 循环异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JarvisCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🚪 系统已关闭。")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()