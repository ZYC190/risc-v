#!/usr/bin/env python3
"""Execute fixed or phone-defined patrol routes with Nav2."""

import json
import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class HomePatrolNode(Node):
    def __init__(self):
        super().__init__("home_patrol")
        default_file = os.path.join(
            get_package_share_directory("home_patrol"), "config", "waypoints.yaml"
        )
        self.declare_parameter("waypoints_file", default_file)
        self.declare_parameter("command_topic", "/home/patrol/cmd")
        self.declare_parameter("status_topic", "/home/patrol/status")
        self.declare_parameter("navigate_action", "/navigate_to_pose")

        self.waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.command_sub = self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._command_callback,
            10,
        )
        self.navigator = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigate_action").value)
        )
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_callback,
            costmap_qos,
        )

        self.running = False
        self.stop_requested = False
        self.current_index = -1
        self.retry_count = 0
        self.goal_handle = None
        self.goal_pending = False
        self.inspection_timer = None
        self.waypoints = []
        self.frame_id = "map"
        self.inspection_seconds = 3.0
        self.max_retries = 0
        self.continue_on_failure = True
        self.near_goal_tolerance = 0.55
        self.near_goal_hold_seconds = 1.0
        self.navigation_timeout_seconds = 60.0
        self.stall_timeout_seconds = 15.0
        self.last_feedback_at = 0.0
        self.goal_started_at = 0.0
        self.best_position_distance = None
        self.progress_anchor_position = None
        self.last_progress_at = 0.0
        self.near_goal_started_at = None
        self.near_goal_cancel_requested = False
        self.timeout_cancel_requested = False
        self.timeout_failure_reason = ""
        self.checked_rooms = []
        self.failed_rooms = []
        self.global_costmap = None
        self.last_robot_position = None
        self.watchdog_timer = self.create_timer(1.0, self._watch_navigation)
        self._publish_status("idle", "巡查节点已启动，等待手机指令", "")

    def _command_callback(self, msg):
        command = msg.data.strip().upper()
        payload = None
        try:
            payload = json.loads(msg.data)
            command = str(payload.get("command", payload.get("cmd", command))).upper()
        except (json.JSONDecodeError, AttributeError, TypeError):
            pass

        if command == "START":
            self._start_patrol()
        elif command == "START_PATH":
            try:
                waypoints = self._parse_waypoints(payload.get("waypoints"))
            except (AttributeError, KeyError, TypeError, ValueError) as exc:
                self._publish_status("invalid_path", str(exc), "")
                return
            self._start_patrol(custom_waypoints=waypoints)
        elif command == "SET_ROOM":
            try:
                waypoint = self._parse_waypoints([payload.get("waypoint")])[0]
            except (AttributeError, KeyError, TypeError, ValueError) as exc:
                self._publish_status("invalid_room", str(exc), "")
                return
            self._save_room_waypoint(waypoint)
        elif command == "GOTO_ROOM":
            try:
                room = str(payload.get("room", "")).strip()
                waypoint = self._load_room_waypoint(room)
            except (
                AttributeError,
                KeyError,
                TypeError,
                ValueError,
                OSError,
                yaml.YAMLError,
            ) as exc:
                self._publish_status("room_not_configured", str(exc), "")
                return
            self._start_patrol(custom_waypoints=[waypoint])
        elif command == "STOP":
            self._stop_patrol()
        elif command == "RELOAD":
            self._reload_only()
        else:
            self._publish_status("error", f"不支持的巡查指令：{command}", "")

    @staticmethod
    def _parse_waypoints(raw_waypoints):
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("巡查路径至少需要 1 个巡查点")
        if len(raw_waypoints) > 1000:
            raise ValueError("单条巡查路径最多支持 1000 个巡查点")

        parsed = []
        for index, item in enumerate(raw_waypoints):
            if not isinstance(item, dict):
                raise ValueError(f"第 {index + 1} 个巡查点格式错误")
            try:
                x = float(item["x"])
                y = float(item["y"])
                yaw = float(item.get("yaw", 0.0))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"第 {index + 1} 个巡查点坐标无效") from exc
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError(f"第 {index + 1} 个巡查点坐标无效")
            parsed.append(
                {
                    "name": str(item.get("name", "")).strip()
                    or f"巡查点 {index + 1}",
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "arrival_tolerance": max(
                        0.0, float(item.get("arrival_tolerance", 0.0))
                    ),
                }
            )
        return parsed

    def _load_waypoints(self):
        with open(self.waypoints_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        recorded_waypoints = [
            item
            for item in (config.get("waypoints") or [])
            if bool(item.get("recorded", False))
        ]
        if not recorded_waypoints:
            raise ValueError("家庭航点尚未标定，请先保存至少一个房间位置")
        # 未标定的可选房间不应阻止已经标定好的房间开始巡查，
        # 也不能把默认的 (0, 0) 当成真实目标发送给 Nav2。
        self.waypoints = self._parse_waypoints(recorded_waypoints)
        self.frame_id = str(config.get("frame_id", "map"))
        self._apply_behavior_config(config)

    def _load_room_waypoint(self, room):
        if not room:
            raise ValueError("请选择要前往的家庭房间")
        with open(self.waypoints_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        target = next(
            (
                item
                for item in (config.get("waypoints") or [])
                if str(item.get("name", "")).strip() == room
            ),
            None,
        )
        if target is None:
            raise ValueError(f"家庭地图中没有房间：{room}")
        if not bool(target.get("recorded", False)):
            raise ValueError(f"{room}位置尚未标定，请先在手机上保存该房间位置")
        return self._parse_waypoints([target])[0]

    def _apply_behavior_config(self, config):
        self.inspection_seconds = max(
            0.0, float(config.get("inspection_seconds", 3.0))
        )
        self.max_retries = max(0, int(config.get("max_retries", 0)))
        self.continue_on_failure = bool(config.get("continue_on_failure", True))
        self.near_goal_tolerance = max(
            0.0, float(config.get("near_goal_tolerance", 0.55))
        )
        self.near_goal_hold_seconds = max(
            0.0, float(config.get("near_goal_hold_seconds", 1.0))
        )
        self.navigation_timeout_seconds = max(
            0.0, float(config.get("navigation_timeout_seconds", 60.0))
        )
        self.stall_timeout_seconds = max(
            0.0, float(config.get("stall_timeout_seconds", 15.0))
        )

    def _load_behavior_config(self):
        try:
            with open(self.waypoints_file, "r", encoding="utf-8") as stream:
                self._apply_behavior_config(yaml.safe_load(stream) or {})
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"无法加载巡查行为参数，使用默认值：{exc}")

    def _save_room_waypoint(self, waypoint):
        if self.running:
            self._publish_status(
                "busy", "巡查进行中，不能修改房间位置", self._room_name()
            )
            return
        try:
            with open(self.waypoints_file, "r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            waypoints = config.get("waypoints") or []
            target = next(
                (
                    item
                    for item in waypoints
                    if str(item.get("name", "")).strip() == waypoint["name"]
                ),
                None,
            )
            if target is None:
                names = "、".join(str(item.get("name", "")) for item in waypoints)
                raise ValueError(f"未知房间：{waypoint['name']}；可选：{names}")
            target.update(
                {
                    "x": round(waypoint["x"], 4),
                    "y": round(waypoint["y"], 4),
                    "yaw": round(waypoint["yaw"], 4),
                    "recorded": True,
                }
            )
            config["configured"] = bool(waypoints) and all(
                bool(item.get("recorded", False)) for item in waypoints
            )
            with open(self.waypoints_file, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    config,
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                    default_flow_style=False,
                )
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self._publish_status("room_save_failed", str(exc), waypoint["name"])
            return

        saved_rooms = [
            str(item.get("name", ""))
            for item in waypoints
            if bool(item.get("recorded", False))
        ]
        self._publish_status(
            "room_saved",
            f"已保存{waypoint['name']}巡查位置",
            waypoint["name"],
            configured=bool(config["configured"]),
            saved_rooms=saved_rooms,
            waypoint=waypoint,
        )

    def _reload_only(self):
        if self.running:
            self._publish_status("busy", "巡查进行中，不能重新加载航点", self._room_name())
            return
        try:
            self._load_waypoints()
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self._publish_status("not_configured", str(exc), "")
            return
        self._publish_status("ready", f"已加载 {len(self.waypoints)} 个家庭航点", "")

    def _start_patrol(self, custom_waypoints=None):
        if self.running:
            self._publish_status("busy", "家庭巡查已经在进行中", self._room_name())
            return
        if custom_waypoints is None:
            try:
                self._load_waypoints()
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
                self._publish_status("not_configured", str(exc), "")
                return
        else:
            self.frame_id = "map"
            self.waypoints = custom_waypoints
            self._load_behavior_config()

        if not self.navigator.wait_for_server(timeout_sec=2.0):
            self._publish_status(
                "nav_unavailable", "Nav2 尚未启动，找不到 /navigate_to_pose", ""
            )
            return

        self.running = True
        self.stop_requested = False
        self.current_index = 0
        self.retry_count = 0
        self._reset_goal_tracking()
        self.checked_rooms = []
        self.failed_rooms = []
        self.last_robot_position = None
        self._publish_status(
            "started", f"开始路径巡查，共 {len(self.waypoints)} 个巡查点", self._room_name()
        )
        self._send_current_goal()

    def _send_current_goal(self):
        if not self.running or self.stop_requested:
            return
        self._reset_goal_tracking()
        # Never let feedback from the previous waypoint satisfy the next
        # waypoint's near-goal or blocked-pose checks.
        self.last_robot_position = None
        self.goal_started_at = time.monotonic()
        waypoint = self.waypoints[self.current_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint["x"]
        goal.pose.pose.position.y = waypoint["y"]
        half_yaw = waypoint["yaw"] / 2.0
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)
        self._publish_status("navigating", f"正在前往{waypoint['name']}", waypoint["name"])
        self.goal_pending = True
        future = self.navigator.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self.goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._handle_navigation_failure(f"发送导航目标失败：{exc}")
            return
        if not goal_handle.accepted:
            self._handle_navigation_failure("Nav2 拒绝了导航目标")
            return
        self.goal_handle = goal_handle
        if self.last_progress_at <= 0.0:
            self.last_progress_at = time.monotonic()
        if self.stop_requested or not self.running:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        now = time.monotonic()
        feedback = feedback_msg.feedback
        current_position = feedback.current_pose.pose.position
        self.last_robot_position = (
            float(current_position.x),
            float(current_position.y),
        )
        waypoint = self.waypoints[self.current_index]
        position_distance = math.hypot(
            float(current_position.x) - waypoint["x"],
            float(current_position.y) - waypoint["y"],
        )
        elapsed = now - self.goal_started_at if self.goal_started_at else 0.0
        nav_distance = max(0.0, float(feedback.distance_remaining))
        position_progressed = False
        if self.best_position_distance is None:
            self.best_position_distance = position_distance
        elif position_distance <= self.best_position_distance - 0.03:
            self.best_position_distance = position_distance
            position_progressed = True

        # Count meaningful motion in any direction as progress.  This keeps a
        # legitimate detour, recovery backup, or side-step from being canceled
        # just because its straight-line distance to the goal is temporarily
        # flat or increasing.
        current_xy = self.last_robot_position
        motion_progressed = False
        if self.progress_anchor_position is None:
            self.progress_anchor_position = current_xy
        elif math.hypot(
            current_xy[0] - self.progress_anchor_position[0],
            current_xy[1] - self.progress_anchor_position[1],
        ) >= 0.08:
            self.progress_anchor_position = current_xy
            motion_progressed = True

        if position_progressed or motion_progressed:
            self.last_progress_at = now

        arrival_tolerance = self._current_arrival_tolerance()
        can_accept_near_goal = (
            arrival_tolerance > 0.0
            and elapsed >= 4.0
            and position_distance <= arrival_tolerance
            and nav_distance <= arrival_tolerance + 0.20
            and self.goal_handle is not None
            and not self.near_goal_cancel_requested
            and not self.timeout_cancel_requested
        )
        if can_accept_near_goal:
            if self.near_goal_started_at is None:
                self.near_goal_started_at = now
            elif now - self.near_goal_started_at >= self.near_goal_hold_seconds:
                self.near_goal_cancel_requested = True
                self._publish_status(
                    "navigating",
                    f"已到达{self._room_name()}附近，正在结束导航",
                    self._room_name(),
                    distance_remaining=round(position_distance, 2),
                    nav_distance_remaining=round(nav_distance, 2),
                    position_distance=round(position_distance, 2),
                    arrival_tolerance=arrival_tolerance,
                )
                self.goal_handle.cancel_goal_async()
                return
        else:
            self.near_goal_started_at = None

        if now - self.last_feedback_at < 2.0:
            return
        self.last_feedback_at = now
        eta_seconds = int(feedback.estimated_time_remaining.sec)
        self._publish_status(
            "navigating",
            f"正在前往{self._room_name()}，剩余 {position_distance:.1f} 米",
            self._room_name(),
            distance_remaining=round(position_distance, 2),
            nav_distance_remaining=round(nav_distance, 2),
            eta_seconds=eta_seconds,
        )

    def _result_callback(self, future):
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self._handle_navigation_failure(f"读取导航结果失败：{exc}")
            return
        if self.stop_requested:
            self._finish_stopped()
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._reset_goal_tracking()
            self._begin_inspection()
            return
        if self.near_goal_cancel_requested:
            self._reset_goal_tracking()
            self._begin_inspection(accepted_near_goal=True)
            return
        if self.timeout_cancel_requested:
            reason = self.timeout_failure_reason or (
                f"导航超过 {self.navigation_timeout_seconds:.0f} 秒"
            )
            self._reset_goal_tracking()
            self._handle_navigation_failure(reason)
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._finish_stopped()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._handle_navigation_failure(f"导航失败，状态码 {status}")
            return

    def _begin_inspection(self, accepted_near_goal=False):
        self.retry_count = 0
        message = f"已到达{self._room_name()}，正在检查"
        self._publish_status(
            "inspecting",
            message,
            self._room_name(),
            inspection_seconds=self.inspection_seconds,
            accepted_near_goal=accepted_near_goal,
        )
        if self.inspection_seconds <= 0.0:
            self._finish_inspection()
            return
        self.inspection_timer = self.create_timer(
            self.inspection_seconds, self._finish_inspection
        )

    def _watch_navigation(self):
        if (
            not self.running
            or self.stop_requested
            or self.goal_handle is None
            or self.goal_started_at <= 0.0
            or self.near_goal_cancel_requested
            or self.timeout_cancel_requested
        ):
            return
        now = time.monotonic()
        elapsed = now - self.goal_started_at
        stalled_for = (
            now - self.last_progress_at if self.last_progress_at > 0.0 else 0.0
        )
        if (
            self.stall_timeout_seconds > 0.0
            and self.last_progress_at > 0.0
            and stalled_for >= self.stall_timeout_seconds
        ):
            self.timeout_cancel_requested = True
            self.timeout_failure_reason = (
                f"连续 {self.stall_timeout_seconds:.0f} 秒没有有效进展"
            )
            self._publish_status(
                "retrying",
                f"{self._room_name()}通道持续受阻，正在结束本巡查点",
                self._room_name(),
                stalled_seconds=round(stalled_for, 1),
            )
            self.goal_handle.cancel_goal_async()
            return

        if (
            self.navigation_timeout_seconds <= 0.0
            or elapsed < self.navigation_timeout_seconds
        ):
            return
        self.timeout_cancel_requested = True
        self.timeout_failure_reason = (
            f"导航超过 {self.navigation_timeout_seconds:.0f} 秒"
        )
        self._publish_status(
            "retrying",
            f"前往{self._room_name()}超时，正在结束本巡查点",
            self._room_name(),
            elapsed_seconds=round(elapsed, 1),
        )
        self.goal_handle.cancel_goal_async()

    def _reset_goal_tracking(self):
        self.goal_started_at = 0.0
        self.best_position_distance = None
        self.progress_anchor_position = None
        self.last_progress_at = 0.0
        self.near_goal_started_at = None
        self.near_goal_cancel_requested = False
        self.timeout_cancel_requested = False
        self.timeout_failure_reason = ""
        self.last_feedback_at = 0.0

    def _current_arrival_tolerance(self):
        if 0 <= self.current_index < len(self.waypoints):
            room_tolerance = float(
                self.waypoints[self.current_index].get("arrival_tolerance", 0.0)
            )
            if room_tolerance > 0.0:
                return room_tolerance
        return self.near_goal_tolerance

    def _finish_inspection(self):
        if self.inspection_timer is not None:
            self.inspection_timer.cancel()
            self.destroy_timer(self.inspection_timer)
            self.inspection_timer = None
        if not self.running or self.stop_requested:
            return
        if self._room_name() not in self.checked_rooms:
            self.checked_rooms.append(self._room_name())
        self._publish_status("checked", f"{self._room_name()}检查完成", self._room_name())
        self.current_index += 1
        if self.current_index >= len(self.waypoints):
            self.running = False
            self.current_index = len(self.waypoints) - 1
            self._publish_status("completed", "本轮家庭巡查完成", "")
            return
        self._send_current_goal()

    def _handle_navigation_failure(self, reason):
        self._reset_goal_tracking()
        if not self.running:
            return
        if self.stop_requested:
            self._finish_stopped()
            return

        blocked_pose = self._blocked_robot_pose()
        if blocked_pose is not None:
            if self._room_name() not in self.failed_rooms:
                self.failed_rooms.append(self._room_name())
            self.running = False
            self._publish_status(
                "localization_lost",
                "机器人定位落在地图障碍区域，巡查已暂停；请重新定位后再开始",
                self._room_name(),
                reason=reason,
                robot_x=blocked_pose["x"],
                robot_y=blocked_pose["y"],
                center_cost=blocked_pose["center_cost"],
                blocked_ratio=blocked_pose["blocked_ratio"],
            )
            return

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self._publish_status(
                "retrying",
                f"{self._room_name()}导航失败，正在重试",
                self._room_name(),
                reason=reason,
                retry=self.retry_count,
            )
            self._send_current_goal()
            return

        if self._room_name() not in self.failed_rooms:
            self.failed_rooms.append(self._room_name())
        self._publish_status(
            "room_failed",
            f"无法到达{self._room_name()}：{reason}",
            self._room_name(),
            reason=reason,
        )
        self.retry_count = 0
        if not self.continue_on_failure:
            self.running = False
            self._publish_status("failed", "家庭巡查因导航失败而终止", self._room_name())
            return
        self.current_index += 1
        if self.current_index >= len(self.waypoints):
            self.running = False
            self.current_index = len(self.waypoints) - 1
            self._publish_status("completed_with_errors", "巡查结束，部分巡查点未能到达", "")
            return
        self._send_current_goal()

    def _costmap_callback(self, msg):
        self.global_costmap = msg

    def _blocked_robot_pose(self):
        if self.global_costmap is None or self.last_robot_position is None:
            return None

        x, y = self.last_robot_position
        info = self.global_costmap.info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return None
        map_x = math.floor((x - float(info.origin.position.x)) / resolution)
        map_y = math.floor((y - float(info.origin.position.y)) / resolution)
        if not (0 <= map_x < info.width and 0 <= map_y < info.height):
            return {
                "x": round(x, 3),
                "y": round(y, 3),
                "center_cost": -1,
                "blocked_ratio": 1.0,
            }

        data = self.global_costmap.data
        center_cost = int(data[map_y * info.width + map_x])
        radius = max(2, int(math.ceil(0.15 / resolution)))
        nearby_costs = []
        for cell_y in range(max(0, map_y - radius), min(info.height, map_y + radius + 1)):
            row_offset = cell_y * info.width
            for cell_x in range(
                max(0, map_x - radius), min(info.width, map_x + radius + 1)
            ):
                nearby_costs.append(int(data[row_offset + cell_x]))
        blocked_ratio = (
            sum(cost >= 99 for cost in nearby_costs) / len(nearby_costs)
            if nearby_costs
            else 0.0
        )
        if center_cost < 99 or blocked_ratio < 0.5:
            return None
        return {
            "x": round(x, 3),
            "y": round(y, 3),
            "center_cost": center_cost,
            "blocked_ratio": round(blocked_ratio, 2),
        }

    def _stop_patrol(self):
        if not self.running:
            self._publish_status("idle", "当前没有进行中的巡查", "")
            return
        self.stop_requested = True
        if self.inspection_timer is not None:
            self.inspection_timer.cancel()
            self.destroy_timer(self.inspection_timer)
            self.inspection_timer = None
        self._publish_status("canceling", "正在停止家庭巡查", self._room_name())
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        elif not self.goal_pending:
            self._finish_stopped()

    def _finish_stopped(self):
        self.running = False
        self.stop_requested = False
        self.goal_handle = None
        self.goal_pending = False
        self._reset_goal_tracking()
        self._publish_status("canceled", "家庭巡查已停止", "")

    def _room_name(self):
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]["name"]
        return ""

    def _publish_status(self, state, message, room, **extra):
        payload = {
            "state": state,
            "message": message,
            "room": room,
            "index": self.current_index,
            "total": len(self.waypoints),
            "checked_rooms": list(self.checked_rooms),
            "failed_rooms": list(self.failed_rooms),
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = HomePatrolNode()
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
