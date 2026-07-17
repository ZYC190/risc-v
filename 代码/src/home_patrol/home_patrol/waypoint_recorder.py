#!/usr/bin/env python3
"""Save the robot's current AMCL pose as a named home waypoint."""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("home_waypoint_recorder")
        default_file = os.path.join(
            get_package_share_directory("home_patrol"), "config", "waypoints.yaml"
        )
        self.declare_parameter("room", "")
        self.declare_parameter("waypoints_file", default_file)
        self.declare_parameter("pose_topic", "/amcl_pose")

        self.room = str(self.get_parameter("room").value).strip()
        self.waypoints_file = os.path.realpath(
            str(self.get_parameter("waypoints_file").value)
        )
        pose_topic = str(self.get_parameter("pose_topic").value)
        self.done = False

        if not self.room:
            raise ValueError("缺少房间名称，例如：-p room:=客厅")

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._pose_callback, 10
        )
        self.timeout_timer = self.create_timer(15.0, self._timeout)
        self.get_logger().info(f"等待 {pose_topic}，准备保存房间：{self.room}")

    def _pose_callback(self, msg):
        if self.done:
            return
        self.done = True

        with open(self.waypoints_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        waypoints = config.get("waypoints") or []
        target = next(
            (item for item in waypoints if str(item.get("name", "")) == self.room),
            None,
        )
        if target is None:
            names = "、".join(str(item.get("name", "")) for item in waypoints)
            self.get_logger().error(f"未知房间：{self.room}；可选：{names}")
            return

        pose = msg.pose.pose
        quaternion = pose.orientation
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        yaw = math.atan2(siny_cosp, cosy_cosp)

        target["x"] = round(float(pose.position.x), 4)
        target["y"] = round(float(pose.position.y), 4)
        target["yaw"] = round(float(yaw), 4)
        target["recorded"] = True
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

        self.get_logger().info(
            f"已保存{self.room}：x={target['x']}, y={target['y']}, yaw={target['yaw']}"
        )
        if config["configured"]:
            self.get_logger().info("五个房间已全部标定，家庭巡查安全锁已自动解除")
        else:
            remaining = [
                str(item.get("name", ""))
                for item in waypoints
                if not bool(item.get("recorded", False))
            ]
            self.get_logger().info(f"还需标定：{'、'.join(remaining)}")

    def _timeout(self):
        if self.done:
            return
        self.done = True
        self.get_logger().error("15 秒内未收到 /amcl_pose，请先启动 Nav2 并完成定位")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointRecorder()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
