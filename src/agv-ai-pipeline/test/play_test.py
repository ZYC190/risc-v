from spacemit_audio import play_audio, play_wav
import subprocess
import re

def find_playback_card(target_name="Device [USB Audio Device]"):
    try:
        # 执行 aplay -l 命令
        result = subprocess.run(["aplay", "-l"], capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()

        for line in lines:
            line = line.strip()
            # 匹配 card N: name [desc]
            match = re.match(r"card (\d+): (.+?) \[", line)
            if match:
                card_num, card_name = match.groups()
                if target_name in line:
                    return int(card_num)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run aplay: {e}")
    
    return 2


# record_device = find_audio_card()
play_device = f'plughw:{find_playback_card()},0'

wav_file_path = "tools/feedback_voice/zhengzaiqianwang.wav"
# thread_play = threading.Thread(target=play_audio, args=(wav_file_path,))
# thread_play.start()

# play_audio(wav_file_path)
# play_device = f'plughw:0,0'

play_wav(wav_file_path, device=play_device)
