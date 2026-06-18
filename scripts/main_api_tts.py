import os
import re
import tempfile
import datetime
import requests
from pydub import AudioSegment
from pydub.playback import play
from openai import OpenAI
import speech_recognition as sr
from aip import AipSpeech  # 引入百度官方智能云 SDK

# ==========================================
# ⚠️ 战车核心通信密钥配置（实战部署前必填！）⚠️
# ==========================================
# 1. DeepSeek 云端大脑密钥
DEEPSEEK_API_KEY = "YOUR_DEEPSEEK_API_KEY"

# 2. 百度智能云 语音密钥 (同时用于识别和合成)
BAIDU_APP_ID = "YOUR_BAIDU_APP_ID"  # <--- ⚠️ 这里必须填入你刚才申请的百度 APP ID
BAIDU_API_KEY = "YOUR_BAIDU_API_KEY"
BAIDU_SECRET_KEY = "YOUR_BAIDU_SECRET_KEY"

# 3. 高德地图气象雷达密钥
GAODE_API_KEY = "YOUR_GAODE_API_KEY"  
CITY_CODE = "500000"  
# ==========================================

# 初始化云端大脑连接
client_llm = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com")
client_asr = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
BAIDU_TOKEN = None

# ==========================================
# 🧠 战车记忆库（海马体）初始化
# ==========================================
chat_history = [
    {"role": "system", "content": "初始化人设，稍后将被时间和天气覆盖"}
]


def get_baidu_token():
    """获取百度 API 的通行证 (Access Token)"""
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
    """呼叫百度云端合成极速语音"""
    token = get_baidu_token()
    if not token:
        return None
        
    url = "https://tsn.baidu.com/text2audio"
    payload = {
        'tex': text,
        'tok': token,
        'cuid': 'agv_car_001',
        'ctp': 1,
        'lan': 'zh',
        'spd': 5,    # 语速
        'pit': 5,    # 音调
        'vol': 15,    # 音量
        'per': 4,    # 音色：4代表度丫丫童声
        'aue': 6     # 6代表WAV格式
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
    """同步播放音频（重采样修复版）"""
    try:
        if os.path.exists(audio_file):
            # 1. 读取云端传回来的音频
            audio = AudioSegment.from_file(audio_file, parameters=["-loglevel", "quiet"])
            
            # 2. 增加 10dB 音量
            louder_audio = audio + 15 
            
            # 3. 🔥 核心黑科技：强制重采样到 48000Hz，满足主板声卡的强迫症！
            fixed_audio = louder_audio.set_frame_rate(48000)
            
            # 4. 用 Python 原生播放器播放！
            play(fixed_audio)
        else:
            print(f'❌ 找不到音频文件: {audio_file}')
    except Exception as e:
        print(f"❌ 播放音频时出错: {e}")


def listen_and_recognize():
    """实时语音监听并调用百度API转文字 (替代旧版本地ASR)"""
    recognizer = sr.Recognizer()
    recognizer.pause_threshold = 0.8  
    
    try:
        # ⚠️ 注意：这里沿用了你之前测试成功的麦克风 ID = 2
        with sr.Microphone(device_index=0, sample_rate=16000) as source:
            print("\n" + "="*45)
            print("👂 [环境静音校准...] ")
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
            print("🟢 [滴！请说话！] 🎤 (按 Ctrl+C 强制退出)")
            
            # 开始录音
            audio = recognizer.listen(source, timeout=10, phrase_time_limit=10)

        print("⏳ 正在发往云端识别你的声音...")
        wav_data = audio.get_wav_data()
        
        # 调用百度官方接口 (1537代表普通话模型)
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


def remove_think_tag(text):
    """移除思考标签（防呆设计）"""
    return re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL)


def get_weather():
    """实时呼叫高德气象卫星获取天气"""
    if not GAODE_API_KEY:
        return "指挥官暂未配置气象雷达密钥，无法获取天气。"
        
    url = f"https://restapi.amap.com/v3/weather/weatherInfo?city={CITY_CODE}&key={GAODE_API_KEY}"
    try:
        res = requests.get(url, timeout=3).json()
        if res.get("status") == "1" and len(res.get("lives", [])) > 0:
            weather = res["lives"][0]
            return f"{weather['city']}今天天气{weather['weather']}，气温{weather['temperature']}摄氏度，{weather['winddirection']}风{weather['windpower']}级。"
    except Exception as e:
        print(f"⚠️ 气象雷达故障: {e}")
    return "气象卫星连接失败，无法获取实时天气。"


def ask_deepseek_api(prompt):
    """呼叫云端 DeepSeek 大脑获取极速回复（挂载记忆 + 时间 + 天气雷达）"""
    global chat_history
    print("🧠 正在呼叫 DeepSeek 思考...", end='', flush=True)
    
    current_time = datetime.datetime.now().strftime("%Y年%m月%d日 %H点%M分")
    current_weather = get_weather()
    
    system_prompt = (
        f"你是一个装在AGV小车上的智能语音助手。现在的系统时间是：{current_time}。"
        f"当前的实时天气是：{current_weather}。"
        f"请用极其简短、口语化、幽默的中文回答我的问题，每次回答坚决不要超过50个字。"
    )
    chat_history[0]["content"] = system_prompt
    chat_history.append({"role": "user", "content": prompt})
    
    if len(chat_history) > 21: 
        chat_history = [chat_history[0]] + chat_history[-20:]

    try:
        response = client_llm.chat.completions.create(
            model="deepseek-chat",
            messages=chat_history, 
            stream=False
        )
        print(" [思考完毕]")
        ai_reply = response.choices[0].message.content
        chat_history.append({"role": "assistant", "content": ai_reply})
        return ai_reply
        
    except Exception as e:
        print(f"\n❌ 云端呼叫失败: {e}")
        chat_history.pop() 
        return "抱歉，我的大脑连接中断了。"


def main():
    print("\n" + "="*45)
    print("🚀 纯云端语音交互引擎 启动中...")
    print("✅ 系统全部就绪！进入免唤醒实时畅聊模式！")

    while True:
        try:
            # 1. 全自动免按键录音 + 云端转文字
            text = listen_and_recognize()
            if not text:
                continue
                
            print(f'\n👤 你说: {text}')

            # 2. 闪电呼叫 DeepSeek API
            full_cont = ask_deepseek_api(text)
            print(f'🤖 AI回复: {full_cont}\n')

            # 3. 呼叫百度发声 API 并播放
            content = remove_think_tag(full_cont).strip()
            if not content:
                continue
                
            print('🎙️ 正在呼叫云端配音员...')
            output_audio = baidu_tts(content)
            
            if output_audio:
                print('🔊 正在播放语音...')
                play_audio(output_audio) 
            else:
                print('❌ 云端语音合成失败，请检查网络或密钥！')
                
        except KeyboardInterrupt:
            print('\n🚪 收到强制中断信号，退出系统。指挥官再见！')
            break


if __name__ == '__main__':
    main()