#!/usr/bin/env python3
import os
# 🛡️ 战车核心装甲：强行开启底层转换，穿透 Linux 系统的 ALSA 音频防火墙
os.environ["PA_ALSA_PLUGHW"] = "1"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
import speech_recognition as sr
from aip import AipSpeech  # 引入百度官方智能云 SDK

# ==========================================
# ⚠️ 百度语音识别秘钥
APP_ID = '121955325'
API_KEY = 'KfFHSRiT6DX0cXRyvCwRehEg'
SECRET_KEY = '25HwPUFPTAZhqkxYDLCB8FBaFK8s8gzG'
# ==========================================

class VoiceCommander(Node):
    def __init__(self):
        super().__init__('voice_commander_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info("🎤 AI 语音控制中心已启动！麦克风预热中...")
        
        # 激活百度大脑 API 客户端
        self.client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
        
        # 启动后台监听线程
        self.listen_thread = threading.Thread(target=self.listen_and_act)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def send_pose(self, angles):
        """发送机械臂角度指令"""
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position =  [float(a) for a in angles]
        self.publisher_.publish(msg)

    def listen_and_act(self):
        """核心监听与动作分发逻辑（常开防刷屏版）"""
        recognizer = sr.Recognizer()
        
        # 💉 强行注入顺风耳基因：降低能量阈值，只要有一丁点声音就录下来！
        recognizer.energy_threshold = 10
        recognizer.dynamic_energy_threshold = False # 严禁系统自动调高门槛
        
        time.sleep(1)
        
        try:
            # 🌟 核心优化：把打开硬件放在 while 循环外面！
            # 💡 注意：如果你重新插拔了线，这里的 device_index 可能是 0 也可能是 1
            # 如果运行报错，请把这里的 1 改回 0 试试！
            with sr.Microphone(device_index=1) as source:
                self.get_logger().info("\n" + "="*45)
                self.get_logger().info("👂 [底层麦克风通道已锁定，保持常开...]")
                self.get_logger().info("🟢 [请随时下达语音指令！]")
                
                # 开始无限监听循环
                while rclpy.ok():
                    try:
                        # timeout=5：每5秒确认一下有没有声音，没有就默默进入下一次循环
                        audio = recognizer.listen(source, timeout=5, phrase_time_limit=8)
                        
                        self.get_logger().info("⏳ 听到声音！正在发往百度云端进行解码...")
                        
                        # 软件重采样为 16000Hz，完美适配百度云端模型
                        wav_data = audio.get_wav_data(convert_rate=16000)
                        result = self.client.asr(wav_data, 'wav', 16000, {'dev_pid': 1537})
                        
                        # 解析百度返回的结果
                        if result['err_no'] == 0:
                            cmd = result['result'][0]
                            self.get_logger().info(f"🗣️ 成功听懂指令: 【{cmd}】")

                            # ===== 动作中枢 =====
                            if "回正" in cmd:
                                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            elif "点头" in cmd:
                                self.send_pose([0.0, 0.0, 0.0, -1.0, 0.0, 0.0])
                                time.sleep(0.5)
                                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            elif "左" in cmd:
                                self.send_pose([-1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            elif "右" in cmd:
                                self.send_pose([1.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            elif "跳舞" in cmd:
                                self.send_pose([0.5, 0.0, 0.0, -1.0, 0.0, 0.0])
                                time.sleep(0.5)
                                self.send_pose([-0.5, 0.0, 0.0, -2.0, 0.0, 0.0])
                                time.sleep(0.5)
                                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.0])
                            elif "抓" in cmd:
                                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, -0.9]) 
                            elif "松" in cmd:
                                self.send_pose([0.0, 0.0, 0.0, -1.57, 0.0, 0.9]) 
                            else:
                                self.get_logger().warn("❓ 指令不在技能库中，请重新下令。")
                        else:
                            # 没听清就直接忽略，不打印错误刷屏
                            pass 
                            
                    except sr.WaitTimeoutError:
                        # 5秒内没听到声音？直接 pass 继续听！世界清净了！
                        pass
                    except Exception as e:
                        self.get_logger().error(f"⚠️ 识别发生错误: {e}")
                        
        except Exception as e:
            self.get_logger().error(f"❌ 麦克风物理通道开启失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()