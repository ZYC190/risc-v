#!/usr/bin/env python3
import argparse
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


class YoloHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class SharedState:
    def __init__(self, image_path: str, debug_path: str, active: bool):
        self.image_path = image_path
        self.debug_path = debug_path
        self.active = active
        self.started_at = time.time()


def make_handler(state: SharedState):
    class YoloFrameHandler(BaseHTTPRequestHandler):
        server_version = "YoloFrameServer/1.0"

        def _send_headers(self, code: int, content_type: str, length: int = None):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            if length is not None:
                self.send_header("Content-Length", str(length))
            self.end_headers()

        def _send_json(self, data, code: int = 200):
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self._send_headers(code, "application/json; charset=utf-8", len(body))
            self.wfile.write(body)

        def _serve_image(self, image_path: str):
            if not state.active:
                self._send_json({"ok": False, "active": False, "message": "camera stream is stopped"}, 503)
                return
            if not os.path.exists(image_path):
                self._send_json(
                    {
                        "ok": False,
                        "active": state.active,
                        "message": "waiting for yolov8_node to create image",
                        "image": image_path,
                    },
                    503,
                )
                return

            try:
                with open(image_path, "rb") as f:
                    body = f.read()
            except OSError as exc:
                self._send_json({"ok": False, "message": str(exc)}, 500)
                return

            self._send_headers(200, "image/jpeg", len(body))
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "*")
            self.end_headers()

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/snapshot.jpg", "/frame.jpg", "/raw.jpg"):
                self._serve_image(state.image_path)
                return
            if path in ("/debug.jpg", "/detect.jpg"):
                self._serve_image(state.debug_path)
                return
            if path == "/control/start":
                state.active = True
                self._send_json({"ok": True, "active": True})
                return
            if path == "/control/stop":
                state.active = False
                self._send_json({"ok": True, "active": False})
                return
            if path == "/state":
                image_exists = os.path.exists(state.image_path)
                debug_exists = os.path.exists(state.debug_path)
                self._send_json(
                    {
                        "ok": True,
                        "active": state.active,
                        "image": state.image_path,
                        "image_exists": image_exists,
                        "debug": state.debug_path,
                        "debug_exists": debug_exists,
                        "uptime_s": round(time.time() - state.started_at, 1),
                    }
                )
                return
            if path == "/":
                body = (
                    "<html><head><meta charset='utf-8'>"
                    "<meta http-equiv='refresh' content='1'>"
                    "<title>YOLO Camera</title></head>"
                    "<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>"
                    "<div style='padding:10px'>YOLO raw camera stream</div>"
                    "<img src='/snapshot.jpg?t="
                    + str(int(time.time() * 1000))
                    + "' style='width:100%;max-width:960px;image-rendering:auto'>"
                    "</body></html>"
                ).encode("utf-8")
                self._send_headers(200, "text/html; charset=utf-8", len(body))
                self.wfile.write(body)
                return

            self._send_json({"ok": False, "message": "not found"}, 404)

        def log_message(self, fmt, *args):
            print("%s - %s" % (self.address_string(), fmt % args))

    return YoloFrameHandler


def main():
    parser = argparse.ArgumentParser(description="Serve YOLO saved frames without opening the stereo camera.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--image", default="/tmp/yolov8_raw_latest.jpg")
    parser.add_argument("--debug", default="/tmp/yolov8_debug_latest.jpg")
    parser.add_argument("--inactive", action="store_true", help="Start in stopped state.")
    args = parser.parse_args()

    state = SharedState(args.image, args.debug, active=not args.inactive)
    server = YoloHttpServer((args.host, args.port), make_handler(state))
    print(f"YOLO frame server started: http://{args.host}:{args.port}")
    print(f"Raw image: {args.image}")
    print(f"Debug image: {args.debug}")
    print("This server does not open the camera. Run yolov8_node first or at the same time.")
    server.serve_forever()


if __name__ == "__main__":
    main()
