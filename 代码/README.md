# 家庭守护服务机器人代码

本目录保存 K1 MUSE Pi Pro / ROS 2 Humble 比赛机器人当前可复现源码。

## 主要模块

- `src/home_patrol`：开场模式、导航模式、家庭地图和巡查状态机。
- `src/jarvis_voice`：语音交互与空气质量监测。
- `src/robot_mqtt_bridge`：手机 MQTT 与 ROS 2 指令/状态桥接。
- `src/wheeltec_arm_control`：六自由度机械臂串口、视觉抓取和动态偏移调参。
- `src/wheeltec_ui_dashboard`：板载触摸屏界面。
- `src/wheeltec_robot_nav2`：Nav2 启动、地图、定位与控制参数。
- `src/yolov8_ros2`：双目视觉和 RVV 数据处理代码。
- `apps/robot_control_app`：家庭看护 Flutter Android 手机端。

## 启动与停止

前台启动（支持 `Ctrl+C` 完整停止）：

```bash
cd /home/zyc/robot2
./start_competition.sh
```

清理后台或异常残留进程：

```bash
/home/zyc/robot2/stop_competition.sh
```

## 本地密钥

仓库不包含任何 API Key、Token、密码或证书。语音节点只从环境变量读取：

- `DEEPSEEK_API_KEY`
- `BAIDU_APP_ID`
- `BAIDU_API_KEY`
- `BAIDU_SECRET_KEY`
- `GAODE_API_KEY`

板端实际值应保存在未纳入 Git 的 `/home/zyc/robot2/.robot_secrets`，不要提交该文件。

## 编译

```bash
cd /home/zyc/robot2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Flutter 手机端：

```bash
cd 代码/apps/robot_control_app
flutter pub get
flutter build apk
```
