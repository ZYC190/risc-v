#!/usr/bin/env python3
"""
逆运动学求解器 — WHEELTEC 六轴桌面机械臂

公式来源：官方 find_color.py (视觉分拣) 和 arm_grabber.py 实战参数
适配：双目相机直接输入3D笛卡尔坐标
"""

import math


class ArmIKSolver:
    """WHEELTEC 六轴机械臂解析逆运动学求解器。"""

    def __init__(self, link_a, link_b, link_c, link_h, auxiliary_angle):
        """
        Args:
            link_a, link_b, link_c, link_h: 机械臂连杆长度 (米)
            auxiliary_angle: 辅助角 (弧度)
        """
        self.link_a = link_a
        self.link_b = link_b
        self.link_c = link_c
        self.link_h = link_h
        self.auxiliary_angle = auxiliary_angle

        # ---- 基础角（同 arm_grabber.py）----
        val = (self.link_c - self.link_h) / self.link_a
        val = max(-1.0, min(1.0, val))
        self.basic_angle = math.acos(val)

        # ---- 虚拟臂三角形常量 ----
        self.caculate_A = (
            self.link_a * math.sin(self.basic_angle)
            + math.sin(self.auxiliary_angle) * self.link_c
        )
        self.caculate_B = (
            self.link_a * math.cos(self.basic_angle)
            + math.cos(self.auxiliary_angle) * self.link_c
        )
        self.caculate_K = math.sqrt(self.caculate_A ** 2 + self.caculate_B ** 2)
        self.caculate_E = math.atan2(self.caculate_B, self.caculate_A)

    def solve(self, true_x, true_y, target_z=0.0, target_rotation_deg=0.0,
              down_offset=-2.2):
        """
        从目标坐标求解6个关节角度。

        Args:
            true_x, true_y: 机械臂基座坐标 (米)
            target_z: 目标高度 (米)，用于高度补偿
            target_rotation_deg: 目标旋转角 (度)
            down_offset: 下压幅度 (rad)，负值越大下压越猛

        Returns:
            tuple: 6个关节角度 (弧度)
        """
        # ---- 1. 底座旋转 ----
        pedestal_angle = math.atan2(true_x, true_y)

        # ---- 2. 水平距离缩放：值越小手臂弯得越多（不向前伸）----
        horizontal_dist = math.sqrt(true_x ** 2 + true_y ** 2)
        # reach_scale < 1.0 = 手臂更弯/更向回收, > 1.0 = 手臂更伸展
        reach_scale = 0.8
        scaled_dist = horizontal_dist * reach_scale
        caculate_C = scaled_dist - self.link_b

        cos_arg = caculate_C / self.caculate_K
        cos_arg = max(-1.0, min(1.0, cos_arg))
        caculate_D = math.acos(cos_arg)

        arm_angle = self.caculate_E - caculate_D   # 与 arm_grabber.py 中的 caculate_G 相同

        # ---- 3. 手部旋转 ----
        hand_angle_deg = target_rotation_deg + 90.0
        if hand_angle_deg > 45.0:
            hand_angle_deg -= 90.0
        hand_angle = math.radians(hand_angle_deg)

        # ---- 4. 关节映射 (arm_grabber.py 第149-153行) ----
        j1 = pedestal_angle
        j2 = arm_angle + down_offset               # 大臂下压
        j3 = -arm_angle - (down_offset * 0.4)      # 小臂配合展平
        j4 = -0.6                                   # 腕部保持水平
        j5 = hand_angle - pedestal_angle                           # 手部旋转
        j6 = 0.0                                   # 第六轴，夹爪由序列器控制

        return (j1, j2, j3, j4, j5, j6)

    def check_reachable(self, true_x, true_y):
        horizontal_dist = math.sqrt(true_x ** 2 + true_y ** 2)
        min_reach = self.link_b
        max_reach = self.link_b + self.caculate_K
        reachable = (min_reach - 0.005) <= horizontal_dist <= (max_reach + 0.005)
        return reachable, horizontal_dist

    def get_workspace_info(self):
        return {
            'base_angle_rad': self.basic_angle,
            'base_angle_deg': math.degrees(self.basic_angle),
            'min_reach_m': self.link_b,
            'max_reach_m': self.link_b + self.caculate_K,
            'link_a': self.link_a,
            'link_b': self.link_b,
            'link_c': self.link_c,
            'link_h': self.link_h,
            'auxiliary_angle': self.auxiliary_angle,
        }