#!/usr/bin/env python3
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapSnapshotNode(Node):
    def __init__(self):
        super().__init__("map_stream_server")
        self.lock = threading.Lock()
        self.latest_map = None
        self.latest_pose = None
        self.latest_goal = None
        self.latest_jpeg = None
        self.last_map_time = 0.0
        self.last_pose_time = 0.0

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.timer = self.create_timer(0.5, self.render_map)
        self.get_logger().info("地图服务已启动: /map + /amcl_pose -> http://0.0.0.0:8090/map.png")

    def map_callback(self, msg):
        with self.lock:
            self.latest_map = msg
            self.last_map_time = time.time()

    def pose_callback(self, msg):
        with self.lock:
            self.latest_pose = msg.pose.pose
            self.last_pose_time = time.time()

    def goal_callback(self, msg):
        with self.lock:
            self.latest_goal = msg.pose

    @staticmethod
    def yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def world_to_pixel(info, x, y):
        px = int((x - info.origin.position.x) / info.resolution)
        py = int((y - info.origin.position.y) / info.resolution)
        return px, info.height - py

    def draw_pose(self, img, info, pose, color):
        px, py = self.world_to_pixel(info, pose.position.x, pose.position.y)
        if px < 0 or py < 0 or px >= img.shape[1] or py >= img.shape[0]:
            return

        yaw = self.yaw_from_quat(pose.orientation)
        cv2.circle(img, (px, py), 7, color, -1, lineType=cv2.LINE_AA)
        end = (
            int(px + 20 * math.cos(yaw)),
            int(py - 20 * math.sin(yaw)),
        )
        cv2.arrowedLine(img, (px, py), end, color, 3, tipLength=0.35, line_type=cv2.LINE_AA)

    def render_map(self):
        with self.lock:
            grid = self.latest_map
            pose = self.latest_pose
            goal = self.latest_goal

        if grid is None:
            canvas = np.full((520, 520, 3), (17, 24, 39), dtype=np.uint8)
            cv2.putText(
                canvas,
                "WAITING FOR /map",
                (105, 255),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (125, 211, 252),
                2,
                cv2.LINE_AA,
            )
        else:
            info = grid.info
            data = np.array(grid.data, dtype=np.int16).reshape((info.height, info.width))
            gray = np.full(data.shape, 128, dtype=np.uint8)
            gray[data == 0] = 245
            gray[data < 0] = 88
            occupied = data > 50
            gray[occupied] = 25
            mid = (data > 0) & (data <= 50)
            gray[mid] = (220 - data[mid] * 2).astype(np.uint8)
            gray = np.flipud(gray)
            canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            scale = min(900 / max(canvas.shape[1], 1), 900 / max(canvas.shape[0], 1))
            if scale > 1.0:
                canvas = cv2.resize(
                    canvas,
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_NEAREST,
                )
                scaled_info = info
            else:
                scaled_info = info

            if goal is not None:
                self.draw_pose(canvas, scaled_info, goal, (34, 197, 94))
            if pose is not None:
                self.draw_pose(canvas, scaled_info, pose, (0, 0, 255))

            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 36), (13, 17, 23), -1)
            cv2.putText(
                canvas,
                f"ROS2 MAP  {info.width}x{info.height}  res={info.resolution:.3f}m",
                (12, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (125, 211, 252),
                2,
                cv2.LINE_AA,
            )

        ok, encoded = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if ok:
            with self.lock:
                self.latest_jpeg = encoded.tobytes()

    def get_frame(self):
        with self.lock:
            return self.latest_jpeg

    def get_state(self):
        with self.lock:
            return {
                "has_map": self.latest_map is not None,
                "has_pose": self.latest_pose is not None,
                "map_age": round(time.time() - self.last_map_time, 2)
                if self.last_map_time
                else None,
                "pose_age": round(time.time() - self.last_pose_time, 2)
                if self.last_pose_time
                else None,
            }


def make_handler(node):
    class MapHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print("[map-http]", fmt % args)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                body = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>机器人SLAM地图</title>
<style>body{margin:0;background:#0d1117;color:#f0f6fc;font-family:sans-serif}
header{height:46px;display:flex;align-items:center;padding:0 14px;background:#10151d;color:#38bdf8;font-weight:800}
img{display:block;width:100vw;height:calc(100vh - 46px);object-fit:contain;background:#05070a}</style>
</head><body><header>ROS2 SLAM 地图 /map + /amcl_pose</header>
<img id="map" src="/map.png">
<script>setInterval(()=>{document.getElementById('map').src='/map.png?t='+Date.now()},800)</script>
</body></html>"""
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body.encode("utf-8"))
                return

            if parsed.path == "/state":
                body = json.dumps(node.get_state(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path in ("/map.png", "/map.jpg"):
                frame = node.get_frame()
                if frame is None:
                    node.render_map()
                    frame = node.get_frame()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.end_headers()
                self.wfile.write(frame or b"")
                return

            self.send_error(404)

    return MapHandler


def main():
    rclpy.init()
    node = MapSnapshotNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    server = ThreadingHTTPServer(("0.0.0.0", 8090), make_handler(node))
    print("地图网页: http://0.0.0.0:8090/  地图图片: /map.png")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    server.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
