# K1 小车建图导航适配（RPLIDAR C1）

本补丁把进迭时空官方案例的“底盘、雷达、SLAM、地图保存、Nav2”分层结构适配到当前比赛工程，同时保留手机五点巡查、三模式切换、MPPI、速度平滑和碰撞监视器。

官方参考页：<https://docs.bit-brick.com/docs/k1/robot/Robot_development/5.6_Robot_Application_Cases/environmental-perception/slam%26navigation>

## 本车硬件契约

| 项目 | 当前实车 | 官方示例 | 处理方式 |
|---|---|---|---|
| 雷达 | RPLIDAR C1 | YDLidar | 保留 `rplidar_ros` |
| USB 转串口 | CP2102N `10c4:ea60` | CH343/CH9102 | 保留内核 `cp210x`，不安装 CH343 雷达驱动 |
| 雷达设备 | `/dev/wheeltec_lidar` | `/dev/ttyCH343USB*` | 保留现有别名，并提供按实车序列号加固的规则 |
| 波特率 | `460800` | 随 YDLidar 型号 | 固定在硬件 YAML |
| 激光接口 | `/scan`，frame `laser` | 依官方包而定 | Nav2、SLAM、碰撞监视器统一使用现有接口 |
| 底盘 | mini_mec，`/dev/wheeltec_controller` | 轮趣底盘 | 保留现有驱动和 `odom_combined` |

不要安装官方文档中的 YDLidar SDK，也不要复制 `serial=="0001"` 的雷达 udev 规则。它们不匹配本车。

## 改动内容

- `wheeltec_lidar.launch.py`：RPLIDAR C1 的串口、波特率、frame、扫描模式和角度裁剪统一从 `wheeltec_param.yaml` 读取。
- `slam_gmapping.launch.py`、`cartographer.launch.py`：增加 `start_base`、`start_lidar`、`lidar_type` 参数，明确硬件启动所有权。
- `wheeltec_mapping.launch.py`：把 SLAM 与建图专用低速平滑、激光碰撞门组合成唯一推荐的现场入口。
- `mapping_safety.yaml`：建图限速为前进 `0.15m/s`、转向 `0.35rad/s`，硬禁后退和横移，并用静态 `0.22m` 车体安全圈，不依赖尚未启动的局部代价地图。
- `wheeltec_nav2.launch.py`：把雷达型号和硬件 YAML 透传到底层雷达 launch。
- `wheeltec_nav2_for_slam.launch.py`：补齐官方文档中的同名入口，但使用当前 gmapping 和带碰撞监视器的安全速度链。
- `save_map.launch.py`：保留官方式入口名，但内部统一调用安全 `save_map.sh`，不能绕过发布者校验、备份和原子提交。
- `save_map.sh`：先保存到临时目录并校验，再备份旧地图；使用代际唯一 PGM，并以原子替换 YAML 作为提交点，进程中断不会拼出新旧地图对。
- `nav_preflight.sh`：只读检查设备、USB 身份、硬件配置、地图和 ROS 包。
- `verify_lidar_runtime.sh`：只启动 RPLIDAR，核对参数、唯一 `/scan` 发布者和频率后自动停止。
- `robot_mode_lock.sh`：让比赛、建图和雷达自检共享底盘/雷达所有权，阻止并发抢串口或双发速度。
- `start_competition.sh` / `stop_competition.sh`：在共享模式锁内启动、清理并最终确认所有受管进程。
- `harden_navigation_params.py`：原子保留现有比赛调参，只把速度平滑器反向下限设为 `0`，并让恢复服务器只加载 `Wait`，禁止盲区后退/恢复旋转。
- `harden_quaternion_source.py`：把上游仅适用于 32 位 `long` 的快速平方根倒数替换为 `std::sqrt`；避免 riscv64 越界读取/严格别名未定义行为影响 IMU 四元数。
- `build_selected_packages.sh`：只发现活动包，规避 `src/agv_mec` 中的同名参考包；发现零字节/不可执行底盘产物时先清理重建，并在结束前验证底盘、雷达、gmapping 三个可执行文件。
- `install_udev_rule.sh`：即使别名缺失也能按 CP2102N 物理设备发现雷达；交互输入 sudo 密码后安装序列号规则，并复核别名、`0660` 权限和 `dialout` 组。

RPLIDAR 当前仍保留 `90°–270°` 裁剪。驱动源码的角度映射结合 `base_footprint -> laser` 的 `yaw≈π` 表明：机器人后半平面被屏蔽，前半平面保留；因此建图安全参数已硬禁后退和横移。碰撞监视器也消费同一份 `/scan`，现场仍须在 RViz 用障碍物确认前后方向，并让车尾远离墙面后再原地转向；确认前不要关闭或改变裁剪。

现有 udev 规则已经可以启动雷达。若部署时无法无密码执行 sudo，可在板端终端补做一次可选加固：

```bash
/home/zyc/robot2/scripts/install_udev_rule.sh
```

不启动底盘的雷达自检：

```bash
/home/zyc/robot2/scripts/verify_lidar_runtime.sh
```

## 定向构建

完整工作区存在同名包，不能直接执行无范围的 `colcon build`。使用：

```bash
/home/zyc/robot2/scripts/build_selected_packages.sh
```

## 比赛现场：重新建图

1. 停止比赛系统并做空闲检查：

```bash
/home/zyc/robot2/stop_competition.sh
/home/zyc/robot2/scripts/nav_preflight.sh mapping
```

2. 前台启动 gmapping（该命令独占一套底盘和一套 RPLIDAR，并启动建图安全链）：

```bash
/home/zyc/robot2/scripts/start_mapping.sh gmapping
```

如需 Cartographer 备选：

```bash
/home/zyc/robot2/scripts/start_mapping.sh cartographer
```

3. 另开 SSH 终端，启动键盘控制：

```bash
cd /home/zyc/robot2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wheeltec_robot_keyboard wheeltec_keyboard \
  --ros-args -r cmd_vel:=cmd_vel_nav
```

必须保留该 remap；否则键盘会直接发布 `/cmd_vel` 并绕过建图碰撞门。安全链为：

```text
wheeltec_keyboard -> cmd_vel_nav -> velocity_smoother
                  -> cmd_vel_raw -> collision_monitor -> cmd_vel -> 底盘
```

4. SLAM 仍在运行时，用安全脚本保存地图：

```bash
/home/zyc/robot2/scripts/save_map.sh
```

脚本写入：

```text
/home/zyc/robot2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml
/home/zyc/robot2/src/wheeltec_robot_nav2/map/WHEELTEC.<时间_随机标记>.pgm
```

`WHEELTEC.yaml` 会用绝对路径指向这一版不可变 PGM（避免逐文件 symlink-install 找不到新文件）；旧 YAML 及其实际引用的图像自动备份到 `/home/zyc/robot2/map_backups/<时间>_before_WHEELTEC.<随机标记>/`。保存后不需要再次构建。

5. 在建图和键盘终端按 `Ctrl+C`，然后清理并恢复比赛系统：

```bash
/home/zyc/robot2/stop_competition.sh
/home/zyc/robot2/start_competition.sh
```

手机进入模式三后，导航管理器继续使用：

```text
start_base:=false
start_lidar:=true
start_waypoint_cycle:=false
```

因此复用比赛系统常驻底盘，只启动一套 RPLIDAR、AMCL 和 Nav2。

## 独立导航与同时建图导航

不运行比赛系统时，使用持有共享硬件锁的独立导航入口：

```bash
/home/zyc/robot2/scripts/start_navigation.sh
```

同时 gmapping + Nav2 的适配入口同样必须走锁定包装脚本：

```bash
/home/zyc/robot2/scripts/start_slam_navigation.sh
```

不要直接执行这两个底层 `ros2 launch`；直接入口不会持有模式锁，可能与比赛系统重复占用底盘或雷达。

“同时建图导航”只用于调试未知环境，不用于正式五点比赛流程；正式流程应先建图、保存、回到建图原点，再用 AMCL 导航。

## PC 可视化

PC 与 K1 使用相同 ROS Domain、网络互通且已安装对应可视化包时：

```bash
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
```

正式巡查仍由手机 APP 发点；RViz 用于核对地图、TF、激光裁剪方向和代价地图。

## 验收检查

- `/scan` 约 10 Hz，`header.frame_id=laser`。
- TF 只有一条 `base_footprint -> laser`，没有 YDLidar 的重复静态 TF。
- 建图时只有一个底盘节点、一个 RPLIDAR 节点和一个 SLAM 节点。
- 导航时速度链保持 `controller -> velocity_smoother -> collision_monitor -> 底盘`。
- `inflation_radius=0.28m`、碰撞预测和安全边界没有被缩小。
- 新地图启用后，清空并重新设置手机旧五点及所有房间坐标。
- 先单测新点 3、点 4，再跑完整五点。

## 回滚

部署前会把所有目标文件和 udev 规则备份到板端同一个带时间戳的目录。发生回归时先运行 `/home/zyc/robot2/stop_competition.sh`，再按部署清单中的备份路径恢复；不要在机器人位置和 AMCL 位姿未知时远程发送运动目标。
