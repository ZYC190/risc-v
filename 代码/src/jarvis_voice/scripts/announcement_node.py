#!/usr/bin/env python3
"""Lightweight fixed-message speaker used while full voice interaction is off."""

import os
import json
import queue
import subprocess
import tempfile
import threading

import requests
import rclpy
from pydub import AudioSegment
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def load_local_secrets():
    candidates = (
        os.environ.get("ROBOT_SECRETS_FILE", ""),
        os.path.expanduser("~/.robot_secrets"),
        os.path.expanduser("~/robot2/.robot_secrets"),
    )
    for path in candidates:
        if not path or not os.path.exists(path):
            continue
        with open(path, "r", encoding="utf-8") as stream:
            for line in stream:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                os.environ.setdefault(
                    key.strip(), value.strip().strip('"').strip("'")
                )


load_local_secrets()
BAIDU_API_KEY = os.environ.get("BAIDU_API_KEY", "")
BAIDU_SECRET_KEY = os.environ.get("BAIDU_SECRET_KEY", "")


class AnnouncementNode(Node):
    def __init__(self):
        super().__init__("navigation_announcement_node")
        self.token = None
        self.messages = queue.Queue(maxsize=10)
        self.stop_event = threading.Event()
        self.subscription = self.create_subscription(
            String, "/voice_announce", self._message_callback, 10
        )
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self.get_logger().info("导航固定播报节点已启动，等待 /voice_announce")

    def _message_callback(self, msg):
        text = str(msg.data).strip()
        if not text or len(text) > 200:
            return
        announcement = text
        try:
            payload = json.loads(text)
        except (json.JSONDecodeError, TypeError):
            payload = None
        if isinstance(payload, dict) and payload.get("type") == "voice_sequence":
            phrases = [
                str(item).strip()
                for item in payload.get("phrases", [])
                if str(item).strip()
            ]
            if not phrases or len(phrases) > 5 or any(len(item) > 200 for item in phrases):
                self.get_logger().warning("分段播报内容无效，已忽略")
                return
            try:
                pause_seconds = float(payload.get("pause_seconds", 1.0))
            except (TypeError, ValueError):
                pause_seconds = 1.0
            announcement = {
                "phrases": phrases,
                "pause_seconds": min(5.0, max(0.0, pause_seconds)),
            }
        try:
            self.messages.put_nowait(announcement)
        except queue.Full:
            self.get_logger().warning("播报队列已满，忽略过期消息")

    def _get_token(self):
        if self.token:
            return self.token
        if not BAIDU_API_KEY or not BAIDU_SECRET_KEY:
            self.get_logger().error("未配置百度 TTS 密钥，无法播放到达播报")
            return None
        try:
            response = requests.post(
                "https://aip.baidubce.com/oauth/2.0/token",
                params={
                    "grant_type": "client_credentials",
                    "client_id": BAIDU_API_KEY,
                    "client_secret": BAIDU_SECRET_KEY,
                },
                timeout=8,
            )
            response.raise_for_status()
            self.token = response.json().get("access_token")
        except Exception as exc:
            self.get_logger().error(f"获取百度 TTS Token 失败: {exc}")
        return self.token

    def _synthesize(self, text):
        token = self._get_token()
        if not token:
            return None
        try:
            response = requests.post(
                "https://tsn.baidu.com/text2audio",
                data={
                    "tex": text,
                    "tok": token,
                    "cuid": "agv_car_001",
                    "ctp": 1,
                    "lan": "zh",
                    "spd": 5,
                    "pit": 5,
                    "vol": 15,
                    "per": 0,
                    "aue": 6,
                },
                timeout=15,
            )
            if response.headers.get("Content-Type", "").startswith("audio/"):
                with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as wav:
                    wav.write(response.content)
                    return wav.name
            self.get_logger().error(f"百度 TTS 合成失败: {response.text[:160]}")
        except Exception as exc:
            self.get_logger().error(f"百度 TTS 请求失败: {exc}")
        return None

    def _play(self, source_path):
        play_path = None
        try:
            audio = (AudioSegment.from_file(source_path) + 8).set_frame_rate(48000)
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as output:
                play_path = output.name
            audio.export(play_path, format="wav")
            subprocess.run(["aplay", "-q", play_path], check=True, timeout=25)
        finally:
            for path in (source_path, play_path):
                if path:
                    try:
                        os.unlink(path)
                    except OSError:
                        pass

    def _worker(self):
        while not self.stop_event.is_set():
            try:
                announcement = self.messages.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                if isinstance(announcement, dict):
                    phrases = announcement["phrases"]
                    pause_seconds = announcement["pause_seconds"]
                else:
                    phrases = [announcement]
                    pause_seconds = 0.0
                for index, text in enumerate(phrases):
                    self.get_logger().info(
                        f"机器人到达播报 {index + 1}/{len(phrases)}: {text}"
                    )
                    audio_path = self._synthesize(text)
                    if audio_path:
                        self._play(audio_path)
                    if index + 1 < len(phrases) and self.stop_event.wait(pause_seconds):
                        break
            except Exception as exc:
                self.get_logger().error(f"播放到达播报失败: {exc}")
            finally:
                self.messages.task_done()

    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AnnouncementNode()
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
