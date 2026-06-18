#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
import paho.mqtt.client as mqtt
import time
import json

# ==================== 战略坐标本 ====================
# 请把您之前踩点获取的 A 点精准坐标填在这里
TARGET_LOCATIONS = {
    "A_point": {
        "x": 1.1949881315231323,
        "y": -0.02308908849954605,
        "qz": -0.009062070229620103,
        "qw": 0.9999589385985573
    },
    "B_point": {   # 原点 / 人的位置
        "x": 0.33391615748405457,
        "y": -0.3690302073955536,
        "qz": 0.9798887339918632,
        "qw": -0.19954465413992842
    }
}

# 全局 publisher，由 main() 初始化
goal_publisher = None

# ==================== Nav2 导航触发器 ====================
def robot_navigate_to(target_name):
    global goal_publisher

    if target_name not in TARGET_LOCATIONS:
        print(f"❌ 警告：坐标本里没有 {target_name} 的坐标！")
        return
        
    print(f"🚀 接收到战略指令！目标锁定：{target_name}，发布到 /goal_pose！")
    
    # 准备导航航弹
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    
    # 灌入坐标
    coords = TARGET_LOCATIONS[target_name]
    goal_pose.pose.position.x = coords["x"]
    goal_pose.pose.position.y = coords["y"]
    goal_pose.pose.orientation.z = coords["qz"]
    goal_pose.pose.orientation.w = coords["qw"]
    
    # 发布到 /goal_pose topic
    goal_publisher.publish(goal_pose)
    print(f"✅ 目标已发布到 /goal_pose")

# ==================== MQTT 拦截回调函数 ====================
def on_message(client, userdata, msg):
    # 收到手机发来的文本，兼容 JSON 和纯文本
    raw = msg.payload.decode('utf-8')
    print(f"📩 [MQTT 收到手机密信] (raw {len(raw)}B): '{raw[:200]}'")

    # 尝试 JSON 解析，提取 text 字段
    payload_text = raw
    try:
        data = json.loads(raw)
        if isinstance(data, dict) and "text" in data:
            payload_text = data["text"]
            print(f"   📋 JSON 解析成功，提取文字: '{payload_text}'")
    except (json.JSONDecodeError, ValueError):
        pass  # 不是 JSON，直接用原始文本

    # 语义解析（关键词模糊匹配）
    if "回原点" in payload_text or "原点" in payload_text or "return" in payload_text.lower() or "B" in payload_text or "b" in payload_text:
        print("🎯 语义解析成功：返回原点！")
        robot_navigate_to("B_point")
    elif "A" in payload_text or "a" in payload_text or "点" in payload_text:
        print("🎯 语义解析成功：前往A点！")
        robot_navigate_to("A_point")
    else:
        print(f"🤔 收到未知指令 '{payload_text}'，小车按兵不动。")

# ==================== 主函数：启动神经网 ====================
def main():
    global goal_publisher

    # 1. 初始化 ROS2 环境
    rclpy.init()

    # 2. 创建 /goal_pose publisher
    goal_publisher = rclpy.create_node('agv_master_node').create_publisher(
        PoseStamped, '/goal_pose', 10
    )
    print("📡 /goal_pose publisher 已就绪")
    
    # 3. 初始化 MQTT 客户端
    MQTT_BROKER = "127.0.0.1"  # 本地 MQTT Broker
    MQTT_TOPIC = "robot/voice_cmd"
    
    client = mqtt.Client()
    client.on_message = on_message
    
    print("🌐 正在连接云端 MQTT 服务器...")
    client.connect(MQTT_BROKER, 1883, 60)
    client.subscribe(MQTT_TOPIC)
    
    # 3. 开启 MQTT 后台死循环监听（不阻塞主线程）
    client.loop_start()
    print(f"📡 监听成功！死守话题: {MQTT_TOPIC}，等待手机无线派单...")
    
    try:
        # 让主程序保持存活
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("🛑 收到停机指令，正在安全切断全网链接...")
        client.loop_stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()