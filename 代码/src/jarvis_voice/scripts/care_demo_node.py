#!/usr/bin/env python3
"""Lightweight, deterministic dialogue relay for the water-delivery demo."""

import base64
import json
import os
import queue
import signal
import subprocess
import tempfile
import threading
import time

import paho.mqtt.client as mqtt


class CareDemoRelay:
    def __init__(self):
        self.parent_topic = "home/care/parent_talk"
        self.dialogue_topic = "home/care/dialogue"
        self.messages = queue.Queue()
        self.stop_event = threading.Event()
        self.parent_turn = 0
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="water_delivery_demo_dialogue",
        )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        if reason_code == 0:
            client.subscribe(self.parent_topic)
            print("递水演示对话已就绪：等待家长发话", flush=True)
        else:
            print(f"MQTT 连接失败：{reason_code}", flush=True)

    def _on_message(self, _client, _userdata, message):
        try:
            decoded = message.payload.decode("utf-8")
            payload = json.loads(decoded)
        except (UnicodeDecodeError, json.JSONDecodeError):
            payload = {"type": "text", "text": "家长发话"}
        self.messages.put(payload if isinstance(payload, dict) else {})

    def _publish_dialogue(self, role, text):
        payload = json.dumps(
            {
                "role": role,
                "text": text,
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            },
            ensure_ascii=False,
        )
        self.client.publish(self.dialogue_topic, payload, qos=0, retain=False)
        print(f"现场对话：{role} -> {text}", flush=True)

    def _play_parent_audio(self, payload):
        encoded = str(payload.get("audio_base64", "")).strip()
        duration_ms = max(0, int(payload.get("duration_ms", 0) or 0))
        duration_text = f"{duration_ms / 1000.0:.1f}秒" if duration_ms else ""
        self._publish_dialogue("parent", f"[家长原声 {duration_text}]".strip())
        if not encoded or len(encoded) > 2_000_000:
            print("家长录音为空或过大，仅执行演示回复", flush=True)
            return
        try:
            audio = base64.b64decode(encoded, validate=True)
        except (ValueError, TypeError):
            print("家长录音 Base64 无效，仅执行演示回复", flush=True)
            return
        if not audio or len(audio) > 1_500_000:
            return

        source_path = None
        wav_path = None
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".m4a") as source:
                source.write(audio)
                source_path = source.name
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as wav:
                wav_path = wav.name
            subprocess.run(
                [
                    "ffmpeg", "-loglevel", "error", "-y", "-i", source_path,
                    "-filter:a", "volume=2.5", "-ar", "48000", wav_path,
                ],
                check=True,
                timeout=12,
            )
            subprocess.run(["aplay", "-q", wav_path], check=True, timeout=25)
        except Exception as exc:
            print(f"家长录音播放失败：{exc}", flush=True)
        finally:
            for path in (source_path, wav_path):
                if path:
                    try:
                        os.unlink(path)
                    except OSError:
                        pass

    def _handle_parent_talk(self, payload):
        self.parent_turn += 1
        if str(payload.get("type", "")).lower() == "parent_audio":
            self._play_parent_audio(payload)
        else:
            text = str(payload.get("text", "家长发话")).strip() or "家长发话"
            self._publish_dialogue("parent", text)

        if self.parent_turn == 1:
            if self.stop_event.wait(5.0):
                return
            self._publish_dialogue("user", "我想喝瓶水")
        else:
            print("家长再次发话：不再自动补发儿童感谢语", flush=True)

    def run(self):
        self.client.connect("127.0.0.1", 1883, 60)
        self.client.loop_start()
        try:
            while not self.stop_event.is_set():
                try:
                    payload = self.messages.get(timeout=0.25)
                except queue.Empty:
                    continue
                try:
                    self._handle_parent_talk(payload)
                finally:
                    self.messages.task_done()
        finally:
            self.client.loop_stop()
            self.client.disconnect()

    def stop(self, *_args):
        self.stop_event.set()


def main():
    relay = CareDemoRelay()
    signal.signal(signal.SIGINT, relay.stop)
    signal.signal(signal.SIGTERM, relay.stop)
    relay.run()


if __name__ == "__main__":
    main()
