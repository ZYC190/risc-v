#!/usr/bin/env python3
"""Expose the current ROS occupancy map and robot pose over a small HTTP API."""

import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def quaternion_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class MapState:
    def __init__(self):
        self.lock = threading.Lock()
        self.png = None
        self.map = {
            "available": False,
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "version": 0,
        }
        self.robot = {
            "available": False,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "updated_at": 0.0,
        }
        self.navigation_state = "stopped"
        self.navigation_updated_at = 0.0
        self.odom_updated_at = 0.0
        self.robot_is_moving = False
        self.odom = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }
        self.command = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}

    def update_map(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return

        pixels = bytearray(width * height * 3)
        for target_y in range(height):
            source_y = height - 1 - target_y
            source_offset = source_y * width
            target_offset = target_y * width * 3
            for x in range(width):
                occupancy = msg.data[source_offset + x]
                if occupancy < 0:
                    color = (12, 20, 34)
                elif occupancy >= 65:
                    color = (22, 35, 55)
                else:
                    color = (225, 235, 244)
                pixel = target_offset + x * 3
                pixels[pixel : pixel + 3] = bytes(color)

        image = Image.frombytes("RGB", (width, height), bytes(pixels))
        buffer = io.BytesIO()
        image.save(buffer, format="PNG", optimize=True)
        origin = msg.info.origin

        with self.lock:
            self.png = buffer.getvalue()
            self.map = {
                "available": True,
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin": {
                    "x": float(origin.position.x),
                    "y": float(origin.position.y),
                    "yaw": quaternion_yaw(origin.orientation),
                },
                "version": int(self.map["version"]) + 1,
            }

    def update_robot(self, msg):
        pose = msg.pose.pose
        with self.lock:
            self.robot = {
                "available": True,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": quaternion_yaw(pose.orientation),
                "updated_at": time.time(),
            }

    def update_odom(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        is_moving = (
            math.hypot(float(twist.linear.x), float(twist.linear.y)) > 0.015
            or abs(float(twist.angular.z)) > 0.03
        )
        with self.lock:
            self.odom_updated_at = time.time()
            self.robot_is_moving = is_moving
            self.odom = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": quaternion_yaw(pose.orientation),
                "linear_x": float(twist.linear.x),
                "linear_y": float(twist.linear.y),
                "angular_z": float(twist.angular.z),
            }

    def update_command(self, msg):
        with self.lock:
            self.command = {
                "linear_x": float(msg.linear.x),
                "linear_y": float(msg.linear.y),
                "angular_z": float(msg.angular.z),
            }

    def update_navigation_status(self, msg):
        try:
            payload = json.loads(msg.data)
            state = str(payload.get("state", "unknown"))
        except (json.JSONDecodeError, AttributeError, TypeError):
            state = "unknown"
        with self.lock:
            self.navigation_state = state
            self.navigation_updated_at = time.time()

    def json_bytes(self):
        with self.lock:
            robot = dict(self.robot)
            now = time.time()
            updated_at = float(robot.get("updated_at", 0.0))
            pose_age = now - updated_at if updated_at > 0.0 else None
            odom_age = (
                now - self.odom_updated_at if self.odom_updated_at > 0.0 else None
            )
            navigation_age = (
                now - self.navigation_updated_at
                if self.navigation_updated_at > 0.0
                else None
            )
            navigation_active = (
                self.navigation_state
                in {
                    "localizing",
                    "running",
                    "navigation_unavailable",
                    "localization_failed",
                }
                and navigation_age is not None
                and navigation_age <= 5.0
            )
            # AMCL 在机器人静止时不会重复发布相同位姿，旧消息仍然代表
            # 当前正确位置。仅在底盘正在移动而 AMCL 超过 3 秒未更新时，
            # 才判定定位流已失效；导航或里程计离线时也立即隐藏旧坐标。
            localization_stale_while_moving = (
                self.robot_is_moving
                and pose_age is not None
                and pose_age > 3.0
            )
            if (
                pose_age is None
                or not navigation_active
                or odom_age is None
                or odom_age > 3.0
                or localization_stale_while_moving
            ):
                robot["available"] = False
            robot["age_seconds"] = (
                round(max(0.0, pose_age), 2) if pose_age is not None else None
            )
            robot["is_moving"] = self.robot_is_moving
            robot["odom_age_seconds"] = (
                round(max(0.0, odom_age), 2) if odom_age is not None else None
            )
            robot["navigation_state"] = self.navigation_state
            payload = {
                "map": dict(self.map),
                "robot": robot,
                "diagnostics": {
                    "odom": dict(self.odom),
                    "cmd_vel": dict(self.command),
                },
                "server_time": now,
            }
        return json.dumps(payload, ensure_ascii=False).encode("utf-8")

    def png_bytes(self):
        with self.lock:
            return self.png


class MapRequestHandler(BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/map_state.json":
            self._respond(200, "application/json; charset=utf-8", self.state.json_bytes())
        elif path == "/map.png":
            png = self.state.png_bytes()
            if png is None:
                self._respond(503, "text/plain; charset=utf-8", b"Map unavailable")
            else:
                self._respond(200, "image/png", png)
        elif path == "/health":
            self._respond(200, "application/json", b'{"ok":true}')
        else:
            self._respond(404, "text/plain; charset=utf-8", b"Not found")

    def _respond(self, status, content_type, body):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *_args):
        return


class MapHttpServer(Node):
    def __init__(self):
        super().__init__("home_map_http_server")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "navigation_status_topic", "/home/navigation/system_status"
        )

        self.state = MapState()
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.state.update_map,
            map_qos,
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self.state.update_robot,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.state.update_odom,
            20,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.state.update_command,
            20,
        )
        self.navigation_status_sub = self.create_subscription(
            String,
            str(self.get_parameter("navigation_status_topic").value),
            self.state.update_navigation_status,
            10,
        )

        MapRequestHandler.state = self.state
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.httpd = ThreadingHTTPServer((host, port), MapRequestHandler)
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever, name="map-http", daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(f"Map API listening on http://{host}:{port}")

    def destroy_node(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.http_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapHttpServer()
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
