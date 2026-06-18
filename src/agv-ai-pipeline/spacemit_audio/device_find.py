
import subprocess
import re

def find_audio_card(target_name="XFMDPV0018"):
    try:
        # 执行 arecord -l 命令
        result = subprocess.run(["arecord", "-l"], capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()

        for line in lines:
            line = line.strip()
            # 用正则匹配 card N: name
            match = re.match(r"card (\d+): (.+?) \[", line)
            if match:
                card_num, card_name = match.groups()
                if target_name in card_name:
                    return int(card_num)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run arecord: {e}")
    
    return 3

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