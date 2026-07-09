import pyaudio

pa = pyaudio.PyAudio()

# 遍历所有设备并打印信息
for i in range(pa.get_device_count()):
    device_info = pa.get_device_info_by_index(i)
    print(f"Index: {device_info['index']}, Name: {device_info['name']}, "
          f"Input Channels: {device_info['maxInputChannels']}, "
          f"Output Channels: {device_info['maxOutputChannels']}, "
          f"Sample Rate: {device_info['defaultSampleRate']}")

pa.terminate()
