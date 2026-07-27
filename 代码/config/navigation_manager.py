#!/usr/bin/env python3
"""Switch competition services and start Nav2 without duplicating hardware."""

import json
import faulthandler
import math
import os
import shutil
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class NavigationManager(Node):
    def __init__(self):
        super().__init__("home_navigation_manager")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("command_topic", "/home/navigation/system_cmd")
        self.declare_parameter("status_topic", "/home/navigation/system_status")
        self.declare_parameter("startup_timeout", 75.0)
        self.declare_parameter("localization_timeout", 20.0)
        self.declare_parameter("action_unready_timeout", 8.0)
        self.declare_parameter("competition_mode", False)

        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.localization_timeout = float(
            self.get_parameter("localization_timeout").value
        )
        self.action_unready_timeout = max(
            2.0, float(self.get_parameter("action_unready_timeout").value)
        )
        self.competition_mode = bool(self.get_parameter("competition_mode").value)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.command_sub = self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._command_callback,
            10,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom_combined", self._odom_callback, 20
        )
        self.voice_control_pub = self.create_publisher(String, "/voice_trigger", 10)
        self.air_control_pub = self.create_publisher(
            String, "/air_alert_control", 10
        )
        self.esp32_control_pub = self.create_publisher(String, "/esp32_cmd", 10)
        self.patrol_control_pub = self.create_publisher(
            String, "/home/patrol/cmd", 10
        )
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.ros2 = shutil.which("ros2") or "ros2"
        self.nav_pid_file = "/tmp/home_navigation_nav2.pid"
        self._cleanup_stale_nav_process()
        self.processes = {
            "nav": None,
            "sound": None,
            "ui": None,
            "voice": None,
            "announce": None,
            "care": None,
            "air": None,
            "arm": None,
            "vision": None,
        }
        self.process_labels = {
            "nav": "Nav2",
            "sound": "声源定位",
            "ui": "触摸屏",
            "voice": "语音交互",
            "announce": "导航固定播报",
            "care": "递水演示对话",
            "air": "安全预警",
            "arm": "机械臂",
            "vision": "双目视觉",
        }
        self.current_mode = "interaction"
        self.state = "booting"
        self.message = "正在准备开场交互模式"
        self.start_requested_at = 0.0
        self.initial_pose_repeats = 0
        self.next_initial_pose_at = 0.0
        self.localization_started_at = 0.0
        self.localization_good_samples = 0
        self.last_amcl_received_at = 0.0
        self.last_amcl_pose = None
        self.last_odom_pose = None
        self.initial_map_pose = (0.0, 0.0, 0.0)
        self.pending_initial_pose = None
        self.action_unready_since = 0.0
        self.navigation_mode = False
        self.live_interaction_enabled = False
        self.voice_enable_repeats = 0
        self.vision_start_at = 0.0
        self.initial_services_started = False
        self.monitor_timer = self.create_timer(0.25, self._monitor)
        self.status_timer = self.create_timer(2.0, self._publish_heartbeat)
        self.initial_service_timer = self.create_timer(
            1.0, self._start_initial_services
        )
        self._publish_status(self.state, self.message)

    def _start_initial_services(self):
        if self.initial_services_started:
            return
        self.initial_services_started = True
        self.initial_service_timer.cancel()
        if self.competition_mode:
            self._set_mode("interaction", force=True)

    def _command_callback(self, msg):
        command = msg.data.strip().upper()
        payload = {}
        try:
            payload = json.loads(msg.data)
            command = str(payload.get("command", payload.get("cmd", command))).upper()
        except (json.JSONDecodeError, AttributeError, TypeError):
            payload = {}

        if command == "SET_MODE":
            self._set_mode(payload.get("mode", ""))
        elif command == "SET_INITIAL_POSE":
            self._set_initial_pose(payload)
        elif command in {"MODE_INTERACTION", "INTERACTION"}:
            self._set_mode("interaction")
        elif command in {"MODE_WATER", "WATER", "DELIVER_WATER"}:
            self._set_mode("water")
        elif command in {"MODE_PATROL", "PATROL"}:
            self._set_mode("patrol")
        elif command in {"DEMO_INTRO_WEATHER", "DEMO_HOME_ENVIRONMENT"}:
            if self.current_mode != "interaction":
                self.get_logger().warning("固定语音演示仅允许在功能一触发")
                return
            self._publish_string(self.voice_control_pub, command)
            self.get_logger().info(f"已转发功能一固定演示指令: {command}")
        elif command == "ENABLE_VOICE_INTERACTION":
            self._enable_live_interaction()
        elif command in {"START", "RESET_ORIGIN"}:
            self._start_navigation()
        elif command == "STOP":
            self._set_mode("interaction")
        elif command == "STATUS":
            self._publish_status(self.state, self.message)
        else:
            self._publish_status("error", f"不支持的导航系统指令：{command}")

    def _set_initial_pose(self, payload):
        pose = payload.get("pose", payload.get("waypoint", payload))
        try:
            values = (
                float(pose["x"]),
                float(pose["y"]),
                float(pose.get("yaw", 0.0)),
            )
        except (KeyError, TypeError, ValueError):
            self._publish_status("error", "设置初始位置失败：缺少有效的 X、Y、朝向")
            return
        if not all(math.isfinite(value) for value in values):
            self._publish_status("error", "设置初始位置失败：坐标或朝向不是有效数字")
            return

        self.pending_initial_pose = values
        self.initial_map_pose = values
        x, y, yaw = values
        self.get_logger().info(
            f"手机设置地图初始位置：x={x:.2f}, y={y:.2f}, "
            f"yaw={math.degrees(yaw):.0f}°"
        )

        start_navigation = bool(payload.get("start_navigation", True))
        if self.current_mode != "patrol":
            if start_navigation:
                self._set_mode("patrol")
            else:
                self._publish_status("initial_pose_saved", "机器人初始位置已保存")
            return

        if not self._process_running("nav"):
            if start_navigation:
                self._start_navigation()
            else:
                self._publish_status("initial_pose_saved", "机器人初始位置已保存")
            return

        if self.navigator.server_is_ready():
            self._begin_origin_localization()
        else:
            self.start_requested_at = time.monotonic()
            self._publish_status(
                "starting", "初始位置已保存，等待 Nav2 就绪后自动应用"
            )

    def _start_navigation(self):
        if self.current_mode != "patrol":
            self._set_mode("patrol")
            return

        if self._process_running("nav"):
            if self.navigator.server_is_ready():
                self._begin_origin_localization()
            else:
                self._publish_status("starting", "巡查导航正在启动，请稍候")
            return

        started = self._start_process(
            "nav",
            [
                self.ros2,
                "launch",
                "wheeltec_nav2",
                "wheeltec_nav2.launch.py",
                "start_base:=false",
                "start_lidar:=true",
                "start_waypoint_cycle:=false",
            ],
        )
        if not started:
            self._publish_status("error", "无法启动导航系统")
            return

        self.state = "starting"
        self.message = "正在加载新比赛地图、雷达定位与 Nav2"
        self.start_requested_at = time.monotonic()
        self._publish_status(self.state, self.message)

    @staticmethod
    def _normalize_mode(mode):
        value = str(mode).strip().lower()
        aliases = {
            "1": "interaction",
            "opening": "interaction",
            "demo": "interaction",
            "2": "water",
            "deliver": "water",
            "delivery": "water",
            "3": "patrol",
            "navigation": "patrol",
            "nav": "patrol",
        }
        return aliases.get(value, value)

    def _set_mode(self, requested_mode, force=False):
        mode = self._normalize_mode(requested_mode)
        if mode not in {"interaction", "water", "patrol"}:
            self._publish_status("error", f"不支持的比赛模式：{requested_mode}")
            return
        # 手机可能在管理器刚完成 DDS 初始化、1 秒开场定时器尚未触发时
        # 就选择模式。显式选择必须优先，不能随后又被默认模式一覆盖。
        if not force and not self.initial_services_started:
            self.initial_services_started = True
            self.initial_service_timer.cancel()
        if mode == self.current_mode and not force:
            if mode == "patrol":
                self._start_navigation()
            else:
                self._publish_status("ready", self.message)
            return

        previous_mode = self.current_mode
        self.current_mode = mode
        self.navigation_mode = mode == "patrol"
        self.voice_enable_repeats = 0
        self.vision_start_at = 0.0

        if mode != "patrol":
            self._publish_string(self.patrol_control_pub, "STOP")
            self._stop_process("nav")
            self.initial_pose_repeats = 0
            self.localization_good_samples = 0
            self.action_unready_since = 0.0

        if mode == "interaction":
            self.live_interaction_enabled = False
            self._stop_process("announce")
            self._stop_process("care")
            self._stop_process("vision")
            self._stop_process("arm")
            self._stop_process("sound")
            self._start_process(
                "voice",
                [
                    self.ros2,
                    "run",
                    "jarvis_voice",
                    "jarvis_node",
                    "--ros-args",
                    "-p",
                    "disable_microphone:=true",
                ],
            )
            self._start_process(
                "air", [self.ros2, "run", "jarvis_voice", "air_sensor_node"]
            )
            self._start_process(
                "ui",
                [self.ros2, "run", "wheeltec_ui_dashboard", "ui_dashboard"],
            )
            self._publish_string(self.air_control_pub, "LINKAGE_ON")
            self._publish_status("ready", "")
        elif mode == "water":
            self._stop_process("announce")
            self._stop_process("sound")
            self._stop_process("air")
            self._stop_process("voice")
            self._publish_string(self.air_control_pub, "LINKAGE_OFF")
            self._publish_string(self.esp32_control_pub, "ALARM_OFF")
            self._publish_string(self.esp32_control_pub, "FAN_OFF")
            self._publish_string(self.esp32_control_pub, "LIGHT_OFF")
            self._start_process(
                "ui",
                [self.ros2, "run", "wheeltec_ui_dashboard", "ui_dashboard"],
            )
            self._start_process(
                "care", [self.ros2, "run", "jarvis_voice", "care_demo_node"]
            )
            arm_started = self._start_process(
                "arm", [self.ros2, "run", "wheeltec_arm_control", "arm_control"]
            )
            if not arm_started:
                self._publish_status("error", "模式二启动失败：机械臂节点无法启动")
                return
            self.vision_start_at = time.monotonic() + 1.5
            self._publish_status(
                "switching", "正在进入模式二：启动机械臂抓取与双目视觉"
            )
        else:
            self._publish_string(self.patrol_control_pub, "STOP")
            for key in ("sound", "voice", "care", "vision", "arm", "announce"):
                self._stop_process(key)
            self._publish_string(self.air_control_pub, "LINKAGE_OFF")
            self._publish_string(self.esp32_control_pub, "ALARM_OFF")
            self._publish_string(self.esp32_control_pub, "FAN_OFF")
            self._publish_string(self.esp32_control_pub, "LIGHT_OFF")
            self._start_process(
                "ui",
                [self.ros2, "run", "wheeltec_ui_dashboard", "ui_dashboard"],
            )
            self._start_process(
                "air",
                [
                    self.ros2,
                    "run",
                    "jarvis_voice",
                    "air_sensor_node",
                    "--ros-args",
                    "-p",
                    "linkage_enabled:=false",
                ],
            )
            self._publish_string(self.air_control_pub, "LINKAGE_OFF")
            self._start_process(
                "announce",
                [self.ros2, "run", "jarvis_voice", "announcement_node"],
            )
            self._publish_status(
                "switching", "正在进入模式三：启动环境监测、雷达定位与巡查导航"
            )
            self._start_navigation()

        self.get_logger().info(f"比赛模式切换：{previous_mode} -> {mode}")

    def _enable_live_interaction(self):
        """Enable the microphone and sound localization only on explicit request."""
        if self.current_mode != "interaction":
            self.get_logger().warning("麦克风和声源定位仅允许在功能一启动")
            return

        # 语音节点常驻以保留固定演示能力；点击后在进程内开启麦克风，避免
        # 重载语音/AI 依赖造成十几秒空窗。重复发布覆盖用户刚开机就点击、
        # DDS 订阅尚未发现的情况，语音节点收到后会幂等处理。
        self.live_interaction_enabled = True
        sound_started = self._start_process(
            "sound", [self.ros2, "run", "jobot_mic", "myagv_mic_node"]
        )
        voice_started = self._process_running("voice") or self._start_process(
            "voice",
            [
                self.ros2,
                "run",
                "jarvis_voice",
                "jarvis_node",
                "--ros-args",
                "-p",
                "disable_microphone:=true",
            ],
        )
        self.voice_enable_repeats = 80
        self._publish_string(self.voice_control_pub, "ENABLE_LIVE_INTERACTION")
        if sound_started and voice_started:
            self.get_logger().info("功能一麦克风与声源定位已启动")
        else:
            self.get_logger().error("功能一麦克风或声源定位启动失败")

    def _monitor(self):
        self._check_processes()

        if self.voice_enable_repeats > 0:
            if self.current_mode == "interaction" and self.live_interaction_enabled:
                self._publish_string(
                    self.voice_control_pub, "ENABLE_LIVE_INTERACTION"
                )
                self.voice_enable_repeats -= 1
            else:
                self.voice_enable_repeats = 0

        if self.state == "starting":
            if self.navigator.server_is_ready():
                self._begin_origin_localization()
            elif time.monotonic() - self.start_requested_at > self.startup_timeout:
                self._stop_process("nav")
                self._publish_status("error", "导航启动超时，请检查 Nav2 终端日志")

        now = time.monotonic()
        if (
            self.state == "localizing"
            and self.initial_pose_repeats > 0
            and now >= self.next_initial_pose_at
        ):
            self._publish_origin_pose()

        if self.state == "localizing":
            if self.localization_good_samples >= 2:
                self.initial_pose_repeats = 0
                self.state = "running"
                self.message = "模式三已就绪：新比赛地图定位稳定，可以设置巡查点"
                self._publish_status(self.state, self.message)
            elif now - self.localization_started_at > self.localization_timeout:
                self.initial_pose_repeats = 0
                self._publish_status(
                    "localization_failed",
                    "AMCL 定位确认超时，请把机器人放回建图原点后点击重新定位",
                )

        if (
            self.current_mode == "water"
            and self.vision_start_at > 0.0
            and now >= self.vision_start_at
        ):
            self.vision_start_at = 0.0
            started = self._start_process(
                "vision", [self.ros2, "run", "yolov8_ros2", "yolov8_node"]
            )
            if started:
                self._publish_status(
                    "ready",
                    "模式二已就绪：底盘、机械臂抓取、视觉识别和递水演示对话",
                )
            else:
                self._publish_status("error", "模式二启动失败：视觉节点无法启动")

        # DDS discovery can briefly report the action server as unavailable even
        # while Nav2 is alive. Keep probing and recover automatically.
        if self.current_mode == "patrol" and self.state in {
            "running",
            "navigation_unavailable",
        }:
            if self.navigator.server_is_ready():
                self.action_unready_since = 0.0
                if self.state == "navigation_unavailable":
                    self._publish_status(
                        "running",
                        "Nav2 navigation service has recovered and goals can be sent again",
                    )
            elif self.state == "running" and self.action_unready_since == 0.0:
                self.action_unready_since = now
            elif (
                self.state == "running"
                and now - self.action_unready_since > self.action_unready_timeout
            ):
                self._publish_status(
                    "navigation_unavailable",
                    "Nav2 导航服务已停止响应，请停止后重新启动导航",
                )

    def _begin_origin_localization(self):
        self.state = "localizing"
        if self.pending_initial_pose is not None:
            self.initial_map_pose = self.pending_initial_pose
            self.pending_initial_pose = None
            self.get_logger().info("正在应用手机设置的地图初始位置")
        elif self.last_odom_pose is None:
            self.initial_map_pose = (0.0, 0.0, 0.0)
            self.get_logger().warning(
                "尚未收到 /odom_combined，AMCL 暂按建图原点初始化"
            )
        else:
            # 比赛开机时 odom 与新比赛地图的 (0, 0, 0) 对齐。导航启动前
            # 声源定位可能已让底盘旋转或移动，必须保留这段里程计变化；若仍
            # 强制写回零位，规划方向就会与实车方向相反并出现转圈、撞墙。
                self.initial_map_pose = self.last_odom_pose
        x, y, yaw = self.initial_map_pose
        self.message = (
            "Nav2 已启动，正在等待 AMCL 确认当前位置"
            f"（x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.0f}°）"
        )
        self.localization_started_at = time.monotonic()
        self.localization_good_samples = 0
        self.last_amcl_received_at = 0.0
        self.last_amcl_pose = None
        self.initial_pose_repeats = 6
        self.next_initial_pose_at = 0.0
        self.action_unready_since = 0.0
        self._publish_status(self.state, self.message)
        self._publish_origin_pose()

    def _odom_callback(self, msg):
        pose = msg.pose.pose
        orientation = pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        yaw = math.atan2(sin_yaw, cos_yaw)
        values = (float(pose.position.x), float(pose.position.y), float(yaw))
        if all(math.isfinite(value) for value in values):
            self.last_odom_pose = values

    def _amcl_pose_callback(self, msg):
        now = time.monotonic()
        pose = msg.pose.pose
        covariance = msg.pose.covariance
        self.last_amcl_received_at = now
        self.last_amcl_pose = (
            float(pose.position.x),
            float(pose.position.y),
            float(covariance[0]),
            float(covariance[7]),
            float(covariance[35]),
        )
        if self.state != "localizing" or now < self.localization_started_at:
            return

        x, y, x_var, y_var, yaw_var = self.last_amcl_pose
        values = (x, y, x_var, y_var, yaw_var)
        requested_x, requested_y, _ = self.initial_map_pose
        is_good = (
            all(math.isfinite(value) for value in values)
            and math.hypot(x - requested_x, y - requested_y) <= 1.0
            and 0.0 <= x_var <= 0.5
            and 0.0 <= y_var <= 0.5
            and 0.0 <= yaw_var <= 1.0
        )
        self.localization_good_samples = (
            self.localization_good_samples + 1 if is_good else 0
        )

    def _publish_origin_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # 板载 EKF 比 ROS 当前时间常慢数十到一百多毫秒。用稍早的时间戳
        # 保证 TF 缓冲区中已有对应变换，避免 AMCL 丢弃初始位姿。
        msg.header.stamp = (
            self.get_clock().now() - Duration(seconds=0.5)
        ).to_msg()
        x, y, yaw = self.initial_map_pose
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        msg.pose.covariance[0] = 0.10
        msg.pose.covariance[7] = 0.10
        msg.pose.covariance[35] = 0.03
        self.initial_pose_pub.publish(msg)
        self.initial_pose_repeats -= 1
        self.next_initial_pose_at = time.monotonic() + 0.75

    def _check_processes(self):
        for key, process in list(self.processes.items()):
            if process is None:
                continue
            exit_code = process.poll()
            if exit_code is None:
                continue
            self.processes[key] = None
            if key == "nav":
                self._remove_nav_pid_file()
            if key == "nav" and self.state in {
                "starting",
                "localizing",
                "running",
                "localization_failed",
                "navigation_unavailable",
            }:
                self._publish_status("error", f"导航进程已退出，代码 {exit_code}")
            elif key in {"arm", "vision", "sound", "ui", "voice", "announce", "care", "air"} and exit_code != 0:
                self.get_logger().warning(
                    f"{self.process_labels[key]}进程已退出，代码 {exit_code}"
                )

    def _start_process(self, key, command):
        if self._process_running(key):
            return True
        try:
            self.processes[key] = subprocess.Popen(command, start_new_session=True)
            if key == "nav":
                with open(self.nav_pid_file, "w", encoding="utf-8") as stream:
                    stream.write(str(self.processes[key].pid))
            self.get_logger().info(f"已启动{self.process_labels[key]}")
            return True
        except OSError as exc:
            self.processes[key] = None
            self.get_logger().error(f"无法启动{self.process_labels[key]}：{exc}")
            return False

    def _process_running(self, key):
        process = self.processes.get(key)
        return process is not None and process.poll() is None

    def _stop_process(self, key):
        process = self.processes.get(key)
        if process is None:
            if key == "nav":
                self._cleanup_stale_nav_process()
            return
        if process.poll() is None:
            process_group = None
            try:
                process_group = os.getpgid(process.pid)
                stop_signal = signal.SIGTERM if key == "nav" else signal.SIGINT
                os.killpg(process_group, stop_signal)
                # ros2 launch 的父进程可能先退出，但 component_container 或
                # ros2 run 的实际节点仍留在同一进程组。不能只等待父进程，
                # 必须确认整个组消失，否则重复启动会累积孤儿节点和 CPU 占用。
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    process.poll()
                    try:
                        os.killpg(process_group, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.1)
                else:
                    os.killpg(process_group, signal.SIGKILL)
            except OSError:
                pass
            finally:
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    if process_group is not None:
                        try:
                            os.killpg(process_group, signal.SIGKILL)
                        except (OSError, ProcessLookupError):
                            pass
        self.processes[key] = None
        if key == "nav":
            self._remove_nav_pid_file()
        self.get_logger().info(f"已停止{self.process_labels[key]}")

    def _remove_nav_pid_file(self):
        try:
            os.unlink(self.nav_pid_file)
        except FileNotFoundError:
            pass
        except OSError as exc:
            self.get_logger().warning(f"无法清理 Nav2 PID 文件：{exc}")

    def _cleanup_stale_nav_process(self):
        try:
            with open(self.nav_pid_file, "r", encoding="utf-8") as stream:
                pid = int(stream.read().strip())
        except (FileNotFoundError, OSError, TypeError, ValueError):
            self._remove_nav_pid_file()
            return

        try:
            with open(f"/proc/{pid}/cmdline", "rb") as stream:
                command_line = stream.read().replace(b"\x00", b" ").decode(
                    "utf-8", errors="replace"
                )
        except OSError:
            self._remove_nav_pid_file()
            return
        if "wheeltec_nav2.launch.py" not in command_line:
            self.get_logger().warning(
                f"忽略无关的旧 PID {pid}，其进程不是受管 Nav2"
            )
            self._remove_nav_pid_file()
            return

        self.get_logger().warning(f"检测到上次残留的 Nav2（PID {pid}），正在清理")
        try:
            process_group = os.getpgid(pid)
            os.killpg(process_group, signal.SIGTERM)
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.25)
            else:
                os.killpg(process_group, signal.SIGKILL)
        except (OSError, ProcessLookupError) as exc:
            self.get_logger().warning(f"清理残留 Nav2 时返回：{exc}")
        finally:
            self._remove_nav_pid_file()

    @staticmethod
    def _publish_string(publisher, value):
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def _stop_navigation(self, publish_status=True):
        self._stop_process("nav")
        self.initial_pose_repeats = 0
        self.localization_good_samples = 0
        self.last_amcl_received_at = 0.0
        self.last_amcl_pose = None
        self.action_unready_since = 0.0
        self.state = "stopped"
        self.message = "巡查导航已停止"
        if publish_status:
            self._publish_status(self.state, self.message)

    def _publish_status(self, state, message):
        self.state = state
        self.message = message
        payload = {
            "state": state,
            "message": message,
            "origin": {
                "x": self.initial_map_pose[0],
                "y": self.initial_map_pose[1],
                "yaw": self.initial_map_pose[2],
            },
            "mode": self.current_mode,
            "services": {
                key: self._process_running(key) for key in self.processes
            },
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        # rclpy binds one severity to each logging call site. Selecting bound
        # methods dynamically from the same line can raise
        # "Logger severity cannot be changed between calls" and kill this
        # manager exactly when a child process exits.
        if state == "error":
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def _publish_heartbeat(self):
        payload = {
            "state": self.state,
            "message": self.message,
            "origin": {
                "x": self.initial_map_pose[0],
                "y": self.initial_map_pose[1],
                "yaw": self.initial_map_pose[2],
            },
            "mode": self.current_mode,
            "services": {
                key: self._process_running(key) for key in self.processes
            },
            "heartbeat": True,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Nav2 最重且会产生容器子进程，必须最先关闭，确保在 ros2 launch
        # 的关闭宽限期内收干净，避免下次启动误用孤儿 action server。
        for key in (
            "nav", "vision", "arm", "air", "voice", "announce", "care", "ui", "sound"
        ):
            self._stop_process(key)
        self.navigation_mode = False
        if rclpy.ok():
            self._publish_status("stopped", "比赛模式管理器已退出，等待重新启动")
        super().destroy_node()


def main(args=None):
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
