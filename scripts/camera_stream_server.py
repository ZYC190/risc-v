#!/usr/bin/env python3
import argparse
import glob
import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2


class CameraStreamer:
    def __init__(self, device, width, height, fps, view):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.view = view
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.snapshot_file = "/tmp/robot_camera_latest.jpg"
        self.status = "camera stopped; call /control/start"
        self.running = True
        self.capture_enabled = False
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)

    def start(self):
        self.thread.start()

    def camera_source(self):
        if self.device == "auto":
            matches = sorted(glob.glob("/dev/v4l/by-id/*USB*Camera*video-index0"))
            if matches:
                resolved = os.path.realpath(matches[0])
                match = re.search(r"video(\d+)$", resolved)
                if match:
                    return int(match.group(1))
                return resolved
            for candidate in (21, 20):
                if os.path.exists(f"/dev/video{candidate}"):
                    return candidate
            return 20
        if isinstance(self.device, str) and self.device.isdigit():
            return int(self.device)
        if isinstance(self.device, str):
            match = re.search(r"video(\d+)$", os.path.realpath(self.device))
            if match:
                return int(match.group(1))
        return self.device

    def camera_path(self):
        source = self.camera_source()
        if isinstance(source, int):
            return f"/dev/video{source}"
        return str(source)

    def select_view(self, frame):
        if self.view == "left" and frame.shape[1] >= 2:
            return frame[:, : frame.shape[1] // 2]
        if self.view == "right" and frame.shape[1] >= 2:
            return frame[:, frame.shape[1] // 2 :]
        return frame

    def enable_capture(self):
        with self.lock:
            self.capture_enabled = True
            self.status = "camera start requested"
        return self.get_state()

    def disable_capture(self):
        with self.lock:
            self.capture_enabled = False
            self.latest_jpeg = None
            self.status = "camera stopped"
        return self.get_state()

    def is_capture_enabled(self):
        with self.lock:
            return self.capture_enabled

    def get_state(self):
        with self.lock:
            return {
                "enabled": self.capture_enabled,
                "status": self.status,
                "has_frame": self.latest_jpeg is not None,
                "fps": self.fps,
                "view": self.view,
            }

    def capture_loop(self):
        while self.running:
            if not self.is_capture_enabled():
                time.sleep(0.1)
                continue

            device_path = self.camera_path()
            if not os.path.exists(device_path):
                with self.lock:
                    self.latest_jpeg = None
                    self.status = f"cannot find {device_path}; retrying"
                print(f"[camera] cannot find {device_path}; retry in 2s")
                time.sleep(2)
                continue

            with self.lock:
                self.status = f"opening {device_path}"
            print(f"[camera] opencv opening {device_path} {self.width}x{self.height}@{self.fps}")

            cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not cap.isOpened():
                with self.lock:
                    self.latest_jpeg = None
                    self.status = f"cannot open {device_path}; retrying"
                print(f"[camera] cannot open {device_path}; retry in 2s")
                cap.release()
                time.sleep(2)
                continue

            failed_reads = 0
            last_frame_at = time.time()
            while self.running and self.is_capture_enabled():
                ok, frame = cap.read()
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads >= 20 or time.time() - last_frame_at > 4:
                        print(f"[camera] read timeout on {device_path}; restarting capture")
                        break
                    time.sleep(0.05)
                    continue

                failed_reads = 0
                last_frame_at = time.time()
                frame = self.select_view(frame)
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 82],
                )
                if not ok:
                    continue

                jpeg = encoded.tobytes()
                with self.lock:
                    self.latest_jpeg = jpeg
                    self.status = f"streaming {device_path} {frame.shape[1]}x{frame.shape[0]}"

                try:
                    with open(self.snapshot_file, "wb") as file:
                        file.write(jpeg)
                except Exception as exc:
                    print(f"[camera] debug snapshot write failed: {exc}")

                time.sleep(1.0 / max(self.fps, 1))

            cap.release()

            with self.lock:
                self.latest_jpeg = None
                if self.capture_enabled:
                    self.status = "camera stream stopped; retrying"
                else:
                    self.status = "camera stopped"
            time.sleep(1)

    def get_frame(self):
        with self.lock:
            return self.latest_jpeg

    def get_status(self):
        with self.lock:
            return self.status


def make_handler(streamer):
    class StreamHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print("[http]", fmt % args)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                status = streamer.get_status()
                state = streamer.get_state()
                button = "停止采集" if state["enabled"] else "开始采集"
                command = "stop" if state["enabled"] else "start"
                body = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>机器人摄像头</title>
  <style>
    body {{
      margin: 0;
      background: #0d1117;
      color: #f0f6fc;
      font-family: system-ui, sans-serif;
    }}
    header {{
      padding: 12px 16px;
      background: #161b22;
      color: #00f0ff;
      font-weight: 700;
    }}
    img {{
      display: block;
      width: 100vw;
      max-height: calc(100vh - 48px);
      object-fit: contain;
      background: #000;
    }}
    p {{
      padding: 8px 16px;
      color: #8b949e;
    }}
    button {{
      margin: 8px 16px;
      padding: 10px 16px;
      border: 0;
      border-radius: 8px;
      background: #00f0ff;
      color: #020617;
      font-weight: 700;
    }}
  </style>
</head>
<body>
  <header>儿童看护模式 · 机器人摄像头</header>
  <img id="camera" src="/snapshot.jpg" alt="camera stream">
  <p>状态：{status} · 单目画面 · 自动刷新</p>
  <button onclick="fetch('/control/{command}').then(() => location.reload())">{button}</button>
  <script>
    const img = document.getElementById('camera');
    setInterval(() => {{
      img.src = '/snapshot.jpg?t=' + Date.now();
    }}, 100);
  </script>
</body>
</html>"""
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body.encode("utf-8"))))
                self.end_headers()
                self.wfile.write(body.encode("utf-8"))
                return

            if parsed.path.startswith("/control/"):
                command = parsed.path.rsplit("/", 1)[-1]
                if command == "start":
                    state = streamer.enable_capture()
                elif command == "stop":
                    state = streamer.disable_capture()
                else:
                    self.send_error(400, "unknown camera command")
                    return

                body = json.dumps(state, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path == "/status":
                body = json.dumps(streamer.get_state(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path.startswith("/snapshot.jpg"):
                frame = streamer.get_frame()
                if frame is None:
                    message = streamer.get_status().encode("utf-8")
                    self.send_response(503)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Content-Length", str(len(message)))
                    self.end_headers()
                    self.wfile.write(message)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                return

            if parsed.path != "/video_feed":
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            while True:
                frame = streamer.get_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(1.0 / max(streamer.fps, 1))
                except (BrokenPipeError, ConnectionResetError):
                    break

    return StreamHandler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="auto")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--view", choices=("left", "right", "full"), default="left")
    args = parser.parse_args()

    streamer = CameraStreamer(args.device, args.width, args.height, args.fps, args.view)
    streamer.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(streamer))
    print(f"[http] open http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        streamer.running = False
        server.server_close()


if __name__ == "__main__":
    main()
