#!/usr/bin/env python3
"""
串口协议模块 — WHEELTEC 六轴机械臂 STM32 通信

帧格式 (16字节):
  [0]     0xAA        帧头
  [1-2]   Joint1      int16, 大端, 弧度×1000
  [3-4]   Joint2      int16, 大端, 弧度×1000
  [5-6]   Joint3      int16, 大端, 弧度×1000
  [7-8]   Joint4      int16, 大端, 弧度×1000
  [9-10]  Joint5      int16, 大端, 弧度×1000
  [11-12] Joint6      int16, 大端, 弧度×1000
  [13]    Mode        0x01=默认模式, 0x02=跟随模式
  [14]    Checksum    XOR 校验 (字节0~13)
  [15]    0xBB        帧尾

来源: wheeltec_table_arm.h:36-41, wheeltec_table_arm.cpp:17-57
"""

import struct

# ===========================================
# 协议常量
# ===========================================
FRAME_HEADER = 0xAA
FRAME_TAIL = 0xBB
MODE_DEFAULT = 0x01    # 默认强力PID
MODE_FOLLOWER = 0x02   # 跟随模式柔和PID
ANGLE_SCALE = 1000.0   # 弧度 → int16 缩放因子

FRAME_SIZE = 16
CHECKSUM_START = 0
CHECKSUM_END = 14      # 对字节 0~13 做 XOR

# 关节方向掩码: 若实测发现某关节运动方向与 ROS 实际方向相反，可将对应位置改为 -1
DIRECTION_MASK = [1, 1, 1, 1, 1, 1]


def build_serial_frame(joint_angles, mode=MODE_DEFAULT, direction_mask=None):
    """
    将6个关节角度构建为16字节串口帧。

    Args:
        joint_angles: 包含6个float的序列 (弧度)
        mode: 工作模式, MODE_DEFAULT(0x01)=默认强力PID 或 MODE_FOLLOWER(0x02)=跟随模式柔和PID
        direction_mask: 关节方向掩码, 默认使用全局 DIRECTION_MASK

    Returns:
        bytes: 16字节帧, 可直接写入串口

    Raises:
        ValueError: 如果 joint_angles 长度不是6
    """
    if len(joint_angles) != 6:
        raise ValueError(
            f"joint_angles 必须是6个元素, 收到了 {len(joint_angles)} 个"
        )

    if direction_mask is None:
        direction_mask = DIRECTION_MASK

    if len(direction_mask) != 6:
        raise ValueError(
            f"direction_mask 必须是6个元素, 收到了 {len(direction_mask)} 个"
        )

    frame = bytearray(FRAME_SIZE)

    # 帧头
    frame[0] = FRAME_HEADER

    # 6个关节角度: float → int16 (×1000), 大端存储
    for i, angle in enumerate(joint_angles):
        # 严格执行官方公式：angle_rad * 1000, 应用方向掩码
        target_rad = angle * direction_mask[i]
        val = int(target_rad * ANGLE_SCALE)

        # Python 处理负数转 16 位有符号整型 (short)
        val = val & 0xFFFF

        base_idx = 1 + i * 2  # 关节i的起始字节索引
        # 严格遵循蓝图：大端序（高字节在前，低字节在后）
        frame[base_idx] = (val >> 8) & 0xFF       # 高 8 位
        frame[base_idx + 1] = val & 0xFF           # 低 8 位

    # 模式字节
    frame[13] = mode

    # 严格执行蓝图校验和：从 [0] 到 [13] 连续异或 (XOR)
    checksum = 0
    for i in range(CHECKSUM_END):
        checksum ^= frame[i]
    frame[14] = checksum

    # 帧尾
    frame[15] = FRAME_TAIL

    return bytes(frame)


def format_frame_hex(frame_bytes):
    """
    将帧字节格式化为可读的十六进制字符串, 用于调试。

    Args:
        frame_bytes: 16字节帧

    Returns:
        str: 空格分隔的十六进制字符串
    """
    return ' '.join(f'{b:02X}' for b in frame_bytes)


def parse_angles_from_frame(frame_bytes):
    """
    从帧字节中解析出关节角度 (用于调试和验证)。

    Args:
        frame_bytes: 16字节帧

    Returns:
        list[float]: 6个关节角度 (弧度)
    """
    angles = []
    for i in range(6):
        base_idx = 1 + i * 2
        value = (frame_bytes[base_idx] << 8) | frame_bytes[base_idx + 1]
        # 处理有符号 int16
        if value > 32767:
            value -= 65536
        angles.append(value / ANGLE_SCALE)
    return angles
