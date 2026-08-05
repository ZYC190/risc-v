#!/usr/bin/env python3
"""Lightweight patrol camera, segmented recorder, and cloud upload bridge.

This service intentionally does not load YOLO.  It owns the stereo camera only
while competition mode three is active, crops the left eye for human review,
updates a JPEG snapshot for the phone, and records timestamped MP4 segments.
"""

import argparse
import datetime as dt
import hashlib
import http.client
import json
import os
import re
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, unquote, urlparse

import cv2


CLIP_RE = re.compile(r"^patrol_(\d{8}T\d{6})\.mp4$")


def iso_time(value: dt.datetime) -> str:
    return value.astimezone().isoformat(timespec="seconds")


def atomic_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class CameraService:
    def __init__(self, args):
        self.args = args
        self.storage = Path(args.storage).expanduser().resolve()
        self.storage.mkdir(parents=True, exist_ok=True)
        self.snapshot = Path(args.snapshot).expanduser().resolve()
        self.snapshot.parent.mkdir(parents=True, exist_ok=True)
        self.config_path = Path(args.config).expanduser().resolve()
        self.log_path = self.storage / "patrol_camera.log"
        self.started_at = time.time()
        self.capture_active = False
        self.active_clip = None
        self.stop_event = threading.Event()
        self.upload_wakeup = threading.Event()
        self.lock = threading.RLock()
        self.last_error = ""
        self.last_upload_error = ""
        self.last_uploaded_at = ""
        self.metadata_cache = {}
        self.uploaded = self._load_uploaded()
        self.upload_thread = threading.Thread(
            target=self._upload_loop, name="patrol-cloud-uploader", daemon=True
        )
        self.capture_thread = threading.Thread(
            target=self._capture_loop, name="patrol-camera-capture", daemon=True
        )

    @property
    def uploaded_path(self) -> Path:
        return self.storage / ".uploaded.json"

    def _load_uploaded(self):
        try:
            value = json.loads(self.uploaded_path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save_uploaded(self):
        atomic_json(self.uploaded_path, self.uploaded)

    def read_config(self):
        defaults = {
            "cloud_base_url": "",
            "cloud_token": "",
            "device_id": "family-guardian-robot",
            "local_retention_hours": 24,
        }
        try:
            value = json.loads(self.config_path.read_text(encoding="utf-8"))
            if isinstance(value, dict):
                defaults.update(value)
        except (OSError, ValueError):
            pass
        defaults["cloud_base_url"] = str(defaults["cloud_base_url"]).strip().rstrip("/")
        defaults["cloud_token"] = str(defaults["cloud_token"]).strip()
        defaults["device_id"] = str(defaults["device_id"]).strip() or "family-guardian-robot"
        try:
            defaults["local_retention_hours"] = max(
                1, int(defaults["local_retention_hours"])
            )
        except (TypeError, ValueError):
            defaults["local_retention_hours"] = 24
        return defaults

    def update_cloud_config(self, value):
        current = self.read_config()
        for key in ("cloud_base_url", "cloud_token", "device_id"):
            if key in value:
                current[key] = str(value[key]).strip()
        if "local_retention_hours" in value:
            current["local_retention_hours"] = max(
                1, int(value["local_retention_hours"])
            )
        base_url = str(current["cloud_base_url"]).rstrip("/")
        if base_url:
            parsed = urlparse(base_url)
            if parsed.scheme not in {"http", "https"} or not parsed.netloc:
                raise ValueError("cloud_base_url must be an http(s) URL")
        current["cloud_base_url"] = base_url
        atomic_json(self.config_path, current)
        try:
            os.chmod(self.config_path, 0o600)
        except OSError:
            pass
        self.upload_wakeup.set()
        return {
            "cloud_base_url": current["cloud_base_url"],
            "device_id": current["device_id"],
            "configured": bool(current["cloud_base_url"] and current["cloud_token"]),
        }

    def start(self):
        with self.log_path.open("a", encoding="utf-8") as stream:
            stream.write(
                "\n=== patrol camera start %s ===\n"
                % iso_time(dt.datetime.now().astimezone())
            )
        self.capture_thread.start()
        self.upload_thread.start()

    def _capture_loop(self):
        os.environ.setdefault("OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT", "3")
        try:
            camera_width, camera_height = (
                int(value) for value in self.args.camera_size.lower().split("x", 1)
            )
        except (TypeError, ValueError):
            camera_width, camera_height = 1280, 480
        output_size = (camera_width // 2, camera_height)
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        record_fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        preview_period = 1.0 / max(1, self.args.preview_fps)
        record_period = 1.0 / max(1, self.args.record_fps)

        while not self.stop_event.is_set():
            capture = cv2.VideoCapture(self.args.device, cv2.CAP_V4L2)
            writer = None
            clip_path = None
            segment_wall_started = None
            segment_frames = 0
            consecutive_failures = 0
            segment_started = 0.0
            next_preview = 0.0
            next_record = 0.0
            try:
                capture.set(cv2.CAP_PROP_FOURCC, fourcc)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
                capture.set(cv2.CAP_PROP_FPS, self.args.camera_fps)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not capture.isOpened():
                    raise RuntimeError(f"unable to open {self.args.device}")

                while not self.stop_event.is_set():
                    ok, stereo = capture.read()
                    if not ok or stereo is None or stereo.size == 0:
                        consecutive_failures += 1
                        if consecutive_failures >= 3:
                            raise RuntimeError("camera returned three empty frames")
                        continue
                    consecutive_failures = 0
                    if stereo.shape[1] < 2:
                        raise RuntimeError(f"invalid camera frame shape: {stereo.shape}")
                    left = stereo[:, : stereo.shape[1] // 2]
                    if (left.shape[1], left.shape[0]) != output_size:
                        left = cv2.resize(left, output_size)

                    now = time.monotonic()
                    if writer is None or now - segment_started >= self.args.segment_seconds:
                        if writer is not None:
                            writer.release()
                            self._finalize_clip(
                                clip_path, segment_wall_started, segment_frames
                            )
                        clip_name = dt.datetime.now().astimezone().strftime(
                            "patrol_%Y%m%dT%H%M%S.mp4"
                        )
                        clip_path = self.storage / clip_name
                        writer = cv2.VideoWriter(
                            str(clip_path),
                            record_fourcc,
                            self.args.record_fps,
                            output_size,
                        )
                        if not writer.isOpened():
                            writer.release()
                            writer = None
                            raise RuntimeError("OpenCV MP4 video writer could not start")
                        with self.lock:
                            self.active_clip = clip_path
                        segment_started = now
                        segment_wall_started = dt.datetime.now().astimezone()
                        segment_frames = 0
                        next_record = now

                    if now >= next_preview:
                        encoded, jpeg = cv2.imencode(
                            ".jpg", left, [cv2.IMWRITE_JPEG_QUALITY, 82]
                        )
                        if encoded:
                            temporary = self.snapshot.with_suffix(".tmp.jpg")
                            with temporary.open("wb") as stream:
                                stream.write(jpeg.tobytes())
                            os.replace(temporary, self.snapshot)
                        next_preview = now + preview_period

                    if writer is not None and now >= next_record:
                        writer.write(left)
                        segment_frames += 1
                        next_record = now + record_period

                    with self.lock:
                        self.capture_active = True
                    self.last_error = ""
            except Exception as exc:
                self.last_error = f"{exc}; retrying camera automatically"
                with self.log_path.open("a", encoding="utf-8") as stream:
                    stream.write(
                        f"{iso_time(dt.datetime.now().astimezone())} {self.last_error}\n"
                    )
            finally:
                if writer is not None:
                    writer.release()
                    self._finalize_clip(
                        clip_path, segment_wall_started, segment_frames
                    )
                capture.release()
                with self.lock:
                    self.capture_active = False
                    self.active_clip = None
            self.stop_event.wait(1)

    def stop(self):
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=6)
        self.upload_wakeup.set()
        if self.upload_thread.is_alive():
            self.upload_thread.join(timeout=8)
        try:
            self.snapshot.unlink()
        except FileNotFoundError:
            pass

    def is_recording(self):
        with self.lock:
            return self.capture_active

    def _probe_duration(self, path: Path):
        try:
            result = subprocess.run(
                [
                    "ffprobe",
                    "-v",
                    "error",
                    "-show_entries",
                    "format=duration",
                    "-of",
                    "default=noprint_wrappers=1:nokey=1",
                    str(path),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
            return max(0.0, float(result.stdout.strip()))
        except (OSError, ValueError, subprocess.SubprocessError):
            return 0.0

    @staticmethod
    def _metadata_path(path: Path):
        return path.with_suffix(".json")

    def _finalize_clip(self, path, started_at, frame_count):
        if path is None or started_at is None or frame_count <= 0:
            return None
        duration = frame_count / max(1, self.args.record_fps)
        return self._store_clip_metadata(path, started_at, duration)

    def _store_clip_metadata(self, path: Path, start, duration):
        if duration <= 0.05 or not path.exists():
            return None
        end = start + dt.timedelta(seconds=duration)
        metadata = {
            "id": path.name,
            "filename": path.name,
            "start_time": iso_time(start),
            "end_time": iso_time(end),
            "duration_seconds": round(duration, 2),
            "size_bytes": path.stat().st_size,
            "url": f"/recordings/{quote(path.name)}",
        }
        try:
            atomic_json(self._metadata_path(path), metadata)
        except OSError:
            pass
        with self.lock:
            self.metadata_cache[path.name] = metadata
        return metadata

    def clip_metadata(self, path: Path):
        match = CLIP_RE.match(path.name)
        if not match:
            return None
        try:
            start = dt.datetime.strptime(match.group(1), "%Y%m%dT%H%M%S").replace(
                tzinfo=dt.datetime.now().astimezone().tzinfo
            )
        except ValueError:
            return None
        with self.lock:
            cached = self.metadata_cache.get(path.name)
        if cached is None:
            try:
                cached = json.loads(
                    self._metadata_path(path).read_text(encoding="utf-8")
                )
                if not isinstance(cached, dict) or cached.get("id") != path.name:
                    cached = None
            except (OSError, ValueError):
                cached = None
        if cached is None:
            duration = self._probe_duration(path)
            if duration <= 0.05:
                return None
            cached = self._store_clip_metadata(path, start, duration)
        if cached is None:
            return None
        metadata = dict(cached)
        metadata["size_bytes"] = path.stat().st_size
        metadata["uploaded"] = path.name in self.uploaded
        with self.lock:
            self.metadata_cache[path.name] = dict(cached)
        return metadata

    def closed_clips(self):
        paths = sorted(self.storage.glob("patrol_*.mp4"))
        if self.is_recording() and paths:
            paths = paths[:-1]
        items = []
        for path in paths:
            metadata = self.clip_metadata(path)
            if metadata:
                items.append((path, metadata))
        return items

    def list_clips(self, start=None, end=None):
        items = []
        for _, metadata in self.closed_clips():
            clip_start = dt.datetime.fromisoformat(metadata["start_time"])
            clip_end = dt.datetime.fromisoformat(metadata["end_time"])
            if start is not None and clip_end < start:
                continue
            if end is not None and clip_start > end:
                continue
            items.append(metadata)
        items.sort(key=lambda item: item["start_time"], reverse=True)
        return items

    def _delete_closed_path(self, path: Path):
        with self.lock:
            active = self.active_clip
        if active is not None and path == active:
            raise RuntimeError("active recording cannot be deleted")
        if not path.exists() or not path.is_file():
            return False
        path.unlink()
        try:
            self._metadata_path(path).unlink()
        except FileNotFoundError:
            pass
        with self.lock:
            self.metadata_cache.pop(path.name, None)
        if path.name in self.uploaded:
            self.uploaded.pop(path.name, None)
            self._save_uploaded()
        return True

    def delete_clip(self, name):
        if not CLIP_RE.match(name):
            raise ValueError("invalid recording")
        return 1 if self._delete_closed_path(self.storage / name) else 0

    def delete_all_clips(self):
        deleted = 0
        for path in sorted(self.storage.glob("patrol_*.mp4")):
            try:
                deleted += 1 if self._delete_closed_path(path) else 0
            except RuntimeError:
                # The current segment remains recording and is intentionally
                # omitted from the history list until it closes.
                continue
        return deleted

    def state(self):
        config = self.read_config()
        clips = list(self.storage.glob("patrol_*.mp4"))
        return {
            "ok": True,
            "active": self.is_recording(),
            "recording": self.is_recording(),
            "preview_available": self.snapshot.exists(),
            "camera_device": self.args.device,
            "segment_seconds": self.args.segment_seconds,
            "local_clip_count": len(clips),
            "cloud_configured": bool(
                config["cloud_base_url"] and config["cloud_token"]
            ),
            "cloud_base_url": config["cloud_base_url"],
            "last_uploaded_at": self.last_uploaded_at,
            "last_error": self.last_error,
            "last_upload_error": self.last_upload_error,
            "uptime_seconds": round(time.time() - self.started_at, 1),
        }

    def _upload_loop(self):
        while not self.stop_event.is_set():
            self._upload_once()
            self._cleanup_local()
            self.upload_wakeup.wait(5)
            self.upload_wakeup.clear()
        self._upload_once(include_current=True)
        self._cleanup_local()

    def _upload_once(self, include_current=False):
        config = self.read_config()
        if not config["cloud_base_url"] or not config["cloud_token"]:
            return
        clips = sorted(self.storage.glob("patrol_*.mp4"))
        if self.is_recording() and clips and not include_current:
            clips = clips[:-1]
        for path in clips:
            if path.name in self.uploaded:
                continue
            metadata = self.clip_metadata(path)
            if not metadata:
                continue
            try:
                self._put_clip(path, metadata, config)
                self.uploaded[path.name] = {
                    "uploaded_at": iso_time(dt.datetime.now().astimezone()),
                    "size_bytes": metadata["size_bytes"],
                }
                self._save_uploaded()
                self.last_uploaded_at = self.uploaded[path.name]["uploaded_at"]
                self.last_upload_error = ""
            except Exception as exc:  # keep the local clip and retry after reconnect
                self.last_upload_error = str(exc)
                break

    def _put_clip(self, path: Path, metadata, config):
        parsed = urlparse(config["cloud_base_url"])
        connection_class = (
            http.client.HTTPSConnection
            if parsed.scheme == "https"
            else http.client.HTTPConnection
        )
        connection = connection_class(parsed.hostname, parsed.port, timeout=30)
        base_path = parsed.path.rstrip("/")
        upload_path = f"{base_path}/api/v1/recordings/{quote(path.name)}"
        headers = {
            "Authorization": f"Bearer {config['cloud_token']}",
            "Content-Type": "video/mp4",
            "Content-Length": str(path.stat().st_size),
            "X-Recording-Start": metadata["start_time"],
            "X-Recording-End": metadata["end_time"],
            "X-Device-Id": config["device_id"],
            "X-Content-SHA256": self._sha256(path),
        }
        connection.putrequest("PUT", upload_path)
        for key, value in headers.items():
            connection.putheader(key, value)
        connection.endheaders()
        with path.open("rb") as stream:
            while True:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                connection.send(chunk)
        response = connection.getresponse()
        body = response.read(4096)
        connection.close()
        if response.status not in {200, 201}:
            raise RuntimeError(
                f"cloud upload HTTP {response.status}: {body.decode('utf-8', 'replace')}"
            )

    @staticmethod
    def _sha256(path: Path):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _cleanup_local(self):
        config = self.read_config()
        cutoff = time.time() - config["local_retention_hours"] * 3600
        clips = sorted(self.storage.glob("patrol_*.mp4"))
        active = clips[-1] if self.is_recording() and clips else None
        for path in clips:
            if path == active or path.stat().st_mtime >= cutoff:
                continue
            # Never delete an old clip that has not reached the configured cloud.
            if config["cloud_base_url"] and path.name not in self.uploaded:
                continue
            try:
                path.unlink()
                try:
                    self._metadata_path(path).unlink()
                except OSError:
                    pass
                with self.lock:
                    self.metadata_cache.pop(path.name, None)
            except OSError:
                pass


class PatrolCameraServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, handler, service):
        super().__init__(address, handler)
        self.service = service


class Handler(BaseHTTPRequestHandler):
    server_version = "PatrolCamera/1.0"

    @property
    def service(self):
        return self.server.service

    def _headers(self, code, content_type, length=None, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        if length is not None:
            self.send_header("Content-Length", str(length))
        for key, value in (extra or {}).items():
            self.send_header(key, str(value))
        self.end_headers()

    def _json(self, value, code=200):
        body = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self._headers(code, "application/json; charset=utf-8", len(body))
        self.wfile.write(body)

    def do_OPTIONS(self):
        self._headers(
            204,
            "text/plain",
            0,
            {
                "Access-Control-Allow-Methods": "GET, POST, DELETE, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type, Authorization",
            },
        )

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path in {"/snapshot.jpg", "/frame.jpg", "/raw.jpg"}:
            self._serve_file(self.service.snapshot, "image/jpeg", allow_range=False)
            return
        if path in {"/state", "/api/state", "/health"}:
            self._json(self.service.state())
            return
        if path in {"/control/start", "/control/stop"}:
            # Lifecycle is owned by competition mode three; phone controls only query it.
            self._json(self.service.state())
            return
        if path == "/api/recordings":
            query = parse_qs(parsed.query)
            try:
                start = self._query_time(query.get("start", [""])[0])
                end = self._query_time(query.get("end", [""])[0])
            except ValueError:
                self._json({"ok": False, "message": "invalid start/end time"}, 400)
                return
            self._json(
                {
                    "ok": True,
                    "source": "robot",
                    "recordings": self.service.list_clips(start, end),
                }
            )
            return
        if path.startswith("/recordings/"):
            name = unquote(path[len("/recordings/") :])
            if not CLIP_RE.match(name):
                self._json({"ok": False, "message": "invalid recording"}, 400)
                return
            self._serve_file(self.service.storage / name, "video/mp4", allow_range=True)
            return
        if path == "/":
            body = (
                "<html><head><meta charset='utf-8'><meta http-equiv='refresh' content='1'>"
                "<title>Patrol Camera</title></head><body style='margin:0;background:#111;color:#eee'>"
                "<h3>室内巡查看护画面（不进行目标识别）</h3>"
                f"<img src='/snapshot.jpg?t={int(time.time() * 1000)}' style='width:100%;max-width:800px'>"
                "</body></html>"
            ).encode("utf-8")
            self._headers(200, "text/html; charset=utf-8", len(body))
            self.wfile.write(body)
            return
        self._json({"ok": False, "message": "not found"}, 404)

    def do_DELETE(self):
        path = urlparse(self.path).path
        if path == "/api/recordings":
            try:
                deleted = self.service.delete_all_clips()
                self._json({"ok": True, "deleted": deleted})
            except OSError as exc:
                self._json({"ok": False, "message": str(exc)}, 500)
            return
        prefix = "/api/recordings/"
        if path.startswith(prefix):
            name = unquote(path[len(prefix) :])
            try:
                deleted = self.service.delete_clip(name)
                if not deleted:
                    self._json({"ok": False, "message": "recording not found"}, 404)
                    return
                self._json({"ok": True, "deleted": deleted})
            except ValueError as exc:
                self._json({"ok": False, "message": str(exc)}, 400)
            except RuntimeError as exc:
                self._json({"ok": False, "message": str(exc)}, 409)
            except OSError as exc:
                self._json({"ok": False, "message": str(exc)}, 500)
            return
        self._json({"ok": False, "message": "not found"}, 404)

    def do_POST(self):
        if urlparse(self.path).path != "/api/cloud/config":
            self._json({"ok": False, "message": "not found"}, 404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 16384:
                raise ValueError("invalid request size")
            value = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(value, dict):
                raise ValueError("JSON object required")
            result = self.service.update_cloud_config(value)
            self._json({"ok": True, **result})
        except (ValueError, OSError) as exc:
            self._json({"ok": False, "message": str(exc)}, 400)

    @staticmethod
    def _query_time(value):
        if not value:
            return None
        parsed = dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=dt.datetime.now().astimezone().tzinfo)
        return parsed

    def _serve_file(self, path: Path, content_type, allow_range):
        if not path.exists() or not path.is_file():
            self._json({"ok": False, "message": "not ready"}, 503)
            return
        total = path.stat().st_size
        start = 0
        end = total - 1
        code = 200
        range_header = self.headers.get("Range", "") if allow_range else ""
        if range_header.startswith("bytes="):
            try:
                first, last = range_header[6:].split("-", 1)
                start = int(first) if first else 0
                end = int(last) if last else end
                if start < 0 or end < start or start >= total:
                    raise ValueError
                end = min(end, total - 1)
                code = 206
            except ValueError:
                self._headers(416, "text/plain", 0, {"Content-Range": f"bytes */{total}"})
                return
        length = end - start + 1
        extra = {"Accept-Ranges": "bytes"}
        if code == 206:
            extra["Content-Range"] = f"bytes {start}-{end}/{total}"
        self._headers(code, content_type, length, extra)
        try:
            with path.open("rb") as stream:
                stream.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = stream.read(min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args), flush=True)


def main():
    parser = argparse.ArgumentParser(description="Patrol camera and segmented recorder")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--device", default="/dev/video20")
    parser.add_argument("--camera-size", default="1280x480")
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument("--preview-fps", type=int, default=4)
    parser.add_argument("--record-fps", type=int, default=10)
    parser.add_argument("--segment-seconds", type=int, default=60)
    parser.add_argument("--crf", type=int, default=28)
    parser.add_argument("--snapshot", default="/tmp/patrol_camera_latest.jpg")
    parser.add_argument("--storage", default="/home/zyc/robot2/data/patrol_recordings")
    parser.add_argument("--config", default="/home/zyc/robot2/config/patrol_camera.json")
    args = parser.parse_args()

    service = CameraService(args)
    server = PatrolCameraServer((args.host, args.port), Handler, service)

    def request_stop(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        service.start()
        print(f"Patrol camera started: http://{args.host}:{args.port}", flush=True)
        print(f"Recordings: {service.storage}", flush=True)
        server.serve_forever(poll_interval=0.25)
    finally:
        server.server_close()
        service.stop()
        print("Patrol camera stopped", flush=True)


if __name__ == "__main__":
    main()
