#!/usr/bin/env python3
"""Switch competition services and start Nav2 without duplicating hardware."""

import json
import math
import os
import shutil
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
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
        self.voice_control_pub = self.create_publisher(String, "/voice_trigger", 10)
        self.air_control_pub = self.create_publisher(
            String, "/air_alert_control", 10
        )
        self.esp32_control_pub = self.create_publisher(String, "/esp32_cmd", 10)
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.ros2 = shutil.which("ros2") or "ros2"
        self.nav_pid_file = "/tmp/home_navigation_nav2.pid"
        self._cleanup_stale_nav_process()
        self.processes = {
            "nav": None,
            "mic": None,
            "ui": None,
            "arm": None,
            "vision": None,
        }
        self.process_labels = {
            "nav": "Nav2",
            "mic": "麦克风阵列",
            "ui": "触摸屏",
            "arm": "机械臂",
            "vision": "双目视觉",
        }
        self.state = "stopped"
        self.message = "导航系统尚未启动"
        self.start_requested_at = 0.0
        self.initial_pose_repeats = 0
        self.next_initial_pose_at = 0.0
        self.localization_started_at = 0.0
        self.localization_good_samples = 0
        self.last_amcl_received_at = 0.0
        self.last_amcl_pose = None
        self.action_unready_since = 0.0
        self.navigation_mode = False
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
            self._start_process(
                "mic", [self.ros2, "run", "jobot_mic", "myagv_mic_node"]
            )
            self._start_process(
                "ui",
                [self.ros2, "run", "wheeltec_ui_dashboard", "ui_dashboard"],
            )
            self.get_logger().info(
                "开场演示模式已就绪：触摸屏、声源定位、语音交互、ESP32 与底盘已开启"
            )

    def _command_callback(self, msg):
        command = msg.data.strip().upper()
        try:
            payload = json.loads(msg.data)
            command = str(payload.get("command", payload.get("cmd", command))).upper()
        except (json.JSONDecodeError, AttributeError, TypeError):
            pass

        if command in {"START", "RESET_ORIGIN"}:
            self._start_navigation()
        elif command == "STOP":
            self._stop_navigation()
        elif command == "STATUS":
            self._publish_status(self.state, self.message)
        else:
            self._publish_status("error", f"不支持的导航系统指令：{command}")

    def _start_navigation(self):
        self._enter_navigation_mode()

        if self.navigator.server_is_ready():
            self._begin_origin_localization()
            return

        if self._process_running("nav"):
            self.state = "starting"
            self.message = "导航系统正在启动，请稍候"
            self._publish_status(self.state, self.message)
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
            self._leave_navigation_mode()
            self._publish_status("error", "无法启动导航系统")
            return

        self.state = "starting"
        self.message = "正在快速加载家庭地图与 Nav2"
        self.start_requested_at = time.monotonic()
        self._publish_status(self.state, self.message)

    def _enter_navigation_mode(self):
        if self.navigation_mode:
            return
        self.navigation_mode = True
        self._publish_string(self.air_control_pub, "LINKAGE_OFF")
        self._publish_string(self.esp32_control_pub, "ALARM_OFF")
        self._publish_string(self.esp32_control_pub, "FAN_OFF")
        self._stop_process("mic")
        self._stop_process("ui")
        self.get_logger().info(
            "已切换到导航模式：触摸屏、声源定位和 ESP32 联动已暂停，语音交互继续运行"
        )

    def _leave_navigation_mode(self, restart_interactions=True):
        self._stop_process("vision")
        self._stop_process("arm")
        self.vision_start_at = 0.0
        if not self.navigation_mode:
            return
        self.navigation_mode = False
        if restart_interactions:
            self._publish_string(self.air_control_pub, "LINKAGE_ON")
            if self.competition_mode:
                self._start_process(
                    "mic", [self.ros2, "run", "jobot_mic", "myagv_mic_node"]
                )
                self._start_process(
                    "ui",
                    [self.ros2, "run", "wheeltec_ui_dashboard", "ui_dashboard"],
                )

    def _monitor(self):
        self._check_processes()

        if self.state == "starting":
            if self.navigator.server_is_ready():
                self._begin_origin_localization()
            elif time.monotonic() - self.start_requested_at > self.startup_timeout:
                self._stop_process("nav")
                self._leave_navigation_mode()
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
                self._start_process(
                    "arm", [self.ros2, "run", "wheeltec_arm_control", "arm_control"]
                )
                self.vision_start_at = now + 1.5
                self.message = "定位已确认，正在启动机械臂与相机"
                self._publish_status(self.state, self.message)
            elif now - self.localization_started_at > self.localization_timeout:
                self.initial_pose_repeats = 0
                self._publish_status(
                    "localization_failed",
                    "AMCL 定位确认超时，请把机器人放回建图原点后点击重新定位",
                )

        if (
            self.state == "running"
            and self.vision_start_at > 0.0
            and now >= self.vision_start_at
        ):
            self.vision_start_at = 0.0
            self._start_process(
                "vision", [self.ros2, "run", "yolov8_ros2", "yolov8_node"]
            )
            self.message = "导航、机械臂、相机与语音交互已就绪"
            self._publish_status(self.state, self.message)

        # DDS discovery can briefly report the action server as unavailable even
        # while Nav2 is alive. Keep probing and recover automatically.
        if self.state in {"running", "navigation_unavailable"}:
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
        self.message = "Nav2 已启动，正在等待 AMCL 确认建图原点定位"
        self.localization_started_at = time.monotonic()
        self.localization_good_samples = 0
        self.last_amcl_received_at = 0.0
        self.last_amcl_pose = None
        self.initial_pose_repeats = 6
        self.next_initial_pose_at = 0.0
        self.action_unready_since = 0.0
        self._publish_status(self.state, self.message)
        self._publish_origin_pose()

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
        is_good = (
            all(math.isfinite(value) for value in values)
            and math.hypot(x, y) <= 1.0
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
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.z = math.sin(0.0)
        msg.pose.pose.orientation.w = math.cos(0.0)
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
                self._leave_navigation_mode()
                self._publish_status("error", f"导航进程已退出，代码 {exit_code}")
            elif key in {"arm", "vision", "mic", "ui"} and exit_code != 0:
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
            try:
                process_group = os.getpgid(process.pid)
                stop_signal = signal.SIGTERM if key == "nav" else signal.SIGINT
                os.killpg(process_group, stop_signal)
                process.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except (OSError, ProcessLookupError):
                    pass
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
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
        self._leave_navigation_mode()
        self.state = "stopped"
        self.message = "导航已停止，现场交互与空气联动已恢复"
        if publish_status:
            self._publish_status(self.state, self.message)

    def _publish_status(self, state, message):
        self.state = state
        self.message = message
        payload = {
            "state": state,
            "message": message,
            "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "mode": "navigation" if self.navigation_mode else "interaction",
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
            "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "mode": "navigation" if self.navigation_mode else "interaction",
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
        for key in ("nav", "vision", "arm", "ui", "mic"):
            self._stop_process(key)
        self.navigation_mode = False
        if rclpy.ok():
            self._publish_status("stopped", "导航管理器已退出，等待重新启动")
        super().destroy_node()


def main(args=None):
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
