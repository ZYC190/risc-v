import serial
import time

# ================= 战前配置 =================
SERIAL_PORT = '/dev/air_sensor' # 如果上一步查出来不同，请修改这里
BAUD_RATE = 9600
# ============================================

def decode_airmod(data):
    """
    环境雷达数据解码核心算法 (复刻 C 语言逻辑)
    """
    # 1. 长度校验
    if len(data) != 17:
        return None

    # 2. 校验和计算 (前16个字节求和，取低8位)
    checksum = sum(data[0:16]) & 0xFF

    # 3. 帧头与校验和验证
    if data[0] == 0x3C and data[1] == 0x02 and checksum == data[16]:
        # 数据拼接 (高字节 * 256 + 低字节)
        co2  = (data[2] << 8) | data[3]
        jq   = (data[4] << 8) | data[5]   # 甲醛
        voc  = (data[6] << 8) | data[7]
        pm25 = (data[8] << 8) | data[9]
        pm10 = (data[10] << 8) | data[11]

        # 温度解析 (处理负数与小数)
        temp_minus = 1 if (data[12] & 0x80) else 0
        temp_i = data[12] & 0x7F
        temp_f = data[13]
        temp_float = temp_i + (temp_f * 0.1)
        if temp_minus:
            temp_float = -temp_float

        # 湿度解析
        humi_i = data[14]
        humi_f = data[15]
        humi_float = humi_i + (humi_f * 0.1)

        return {
            "CO2": co2,
            "甲醛": jq,
            "VOC": voc,
            "PM2.5": pm25,
            "PM10": pm10,
            "温度": round(temp_float, 1),
            "湿度": round(humi_float, 1)
        }
    else:
        print("⚠️ 数据帧校验失败，可能受到电磁干扰！")
        return None

# ================= 主循环 =================
try:
    # 建立串行通信连接
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    print(f"🚀 环境雷达已连接至 {SERIAL_PORT}，开始扫描...\n" + "="*40)

    while True:
        # 寻找帧头 0x3C
        if ser.read(1) == b'\x3C':
            # 确认下一个字节是 0x02
            if ser.read(1) == b'\x02':
                # 已经读了2个字节，再读剩下的 15 个字节
                rest_of_data = ser.read(15)
                
                if len(rest_of_data) == 15:
                    # 拼装完整的 17 字节数据帧
                    full_frame = b'\x3C\x02' + rest_of_data
                    
                    # 送入解码引擎
                    result = decode_airmod(full_frame)
                    
                    if result:
                        print(f"🌡️ 温度: {result['温度']}°C | 💧 湿度: {result['湿度']}%")
                        print(f"🌫️ PM2.5: {result['PM2.5']} ug/m3 | 💨 PM10: {result['PM10']} ug/m3")
                        print(f"☣️ 甲醛: {result['甲醛']} ug/m3 | 🧪 VOC: {result['VOC']} ug/m3 | ☁️ CO2: {result['CO2']} ppm")
                        print("-" * 40)
                        
        # 传感器手册通常说明预热时间和发送频率，这里稍微等待防止刷屏太快
        time.sleep(0.5)

except serial.SerialException:
    print(f"❌ 无法打开串口 {SERIAL_PORT}！请检查接线或确认是否加了 sudo 权限。")
except KeyboardInterrupt:
    print("\n🚪 停止扫描，系统退出。")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()