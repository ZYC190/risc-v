import os
import time
import sys


sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


from spacemit_audio import ASRModel
from spacemit_audio.record_loop import RecAudioDB
from spacemit_audio import find_audio_card

record_device = find_audio_card()

print(f"录音设备索引: {record_device}")

rec_audio = RecAudioDB(sld=1, min_db=4000, max_time=10, rate=16000, device_index=record_device)
asr_model = ASRModel()

if __name__ == '__main__':
    try:
        while True:
            print("Press enter to start!")
            input() # enter 触发

            audio_ret = rec_audio.record_audio() # 获取录音文件路径
            if rec_audio.exit_mode == 0:
            
                text = asr_model.generate(audio_ret)
                print('user: ', text)

            else:
                
                pass

    except KeyboardInterrupt:
        print("process was interrupted by user.")
