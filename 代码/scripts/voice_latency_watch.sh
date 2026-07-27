#!/usr/bin/env bash
set -euo pipefail

filter='⏱️|用户说|已识别小薇|人声检测|语音识别失败|ASR异常|持续监听启动失败'

echo "实时显示语音识别耗时；按 Ctrl+C 退出观察，不会停止机器人系统。"
echo "重点看：一句话采集完成、百度ASR、DeepSeek回复、语音合成。"

if systemctl --user is-active --quiet competition-system.service; then
    exec journalctl --user -fu competition-system.service -o cat \
        | grep --line-buffered -E "${filter}"
fi

exec tail -n 200 -F /home/zyc/robot2/competition_system.log \
    | grep --line-buffered -E "${filter}"
