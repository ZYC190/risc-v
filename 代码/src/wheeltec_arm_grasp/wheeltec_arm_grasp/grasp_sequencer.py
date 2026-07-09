#!/usr/bin/env python3
"""
抓取序列状态机 — 多阶段抓取动作编排

状态转移:
  IDLE → APPROACH → GRASP → CLOSE_HAND → LIFT → (PLACE) → RELEASE → RETURN → IDLE

每个状态执行一个动作，通过定时器回调驱动状态转移。
"""

import math
import time
from enum import Enum


class GraspState(Enum):
    IDLE = "idle"
    APPROACH = "approach"        # 预接近: 移到目标上方, 夹爪张开
    GRASP = "grasp"              # 下探: 降到抓取高度
    CLOSE_HAND = "close_hand"    # 闭合夹爪
    LIFT = "lift"                # 抬起物体
    PLACE = "place"              # (可选) 移动到放置位置
    RELEASE = "release"          # 张开夹爪释放
    RETURN = "return"            # 返回观察位姿
    DONE = "done"                # 完成


class GraspSequencer:
    """
    协调多阶段抓取序列。

    通过定时器回调逐步执行抓取动作。
    每个阶段之间需要足够的时间让机械臂完成运动 (sleep)。
    """

    def __init__(
        self,
        ik_solver,
        serial_writer,
        node,
        approach_height_z=0.08,
        lift_height_z=0.10,
        hand_open_angle=0.0,
        hand_close_angle=0.5,
        down_offset=-2.2,
        step_delay=2.0,
    ):
        """
        Args:
            ik_solver: ArmIKSolver 实例
            serial_writer: 可调用对象, 接收6个关节角度发送到串口
            node: ROS 2 节点 (用于日志)
            approach_height_z: 预接近高度 (目标点上方, 米)
            lift_height_z: 抬起高度 (抓取后, 米)
            hand_open_angle: 夹爪张开角度 (弧度)
            hand_close_angle: 夹爪闭合角度 (弧度)
            down_offset: 大臂下压幅度 (负值, rad), 与 arm_grabber.py 一致
            step_delay: 状态间等待时间 (秒)
        """
        self.ik_solver = ik_solver
        self._send = serial_writer
        self._node = node
        self.approach_height_z = approach_height_z
        self.lift_height_z = lift_height_z
        self.hand_open_angle = hand_open_angle
        self.hand_close_angle = hand_close_angle
        self.down_offset = down_offset
        self.step_delay = step_delay

        self.state = GraspState.IDLE
        self.target_point = None     # (x, y, z) 在机械臂基座坐标系
        self._last_step_time = 0.0
        self._current_joints = None  # 缓存当前关节角, 用于仅改夹爪

    @property
    def is_active(self):
        """是否正在执行抓取序列"""
        return self.state not in (GraspState.IDLE, GraspState.DONE)

    def start_grasp(self, target_x, target_y, target_z):
        """
        对检测到的目标启动抓取序列。

        Args:
            target_x, target_y: 机械臂基座坐标系的平面坐标 (米)
            target_z: 目标点高度 (米, 在基座坐标系中通常为负值或0)
        """
        self.target_point = (target_x, target_y, target_z)
        self.state = GraspState.APPROACH
        self._last_step_time = 0.0
        self._node.get_logger().info(
            f'🎯 启动抓取: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}'
        )

    def step(self):
        """
        执行抓取序列中的下一步。

        Returns:
            bool: True 如果序列仍在进行, False 如果已完成
        """
        if self.state == GraspState.IDLE:
            return False

        if self.state == GraspState.DONE:
            self.state = GraspState.IDLE
            return False

        # 执行当前状态的动作
        action_map = {
            GraspState.APPROACH: self._do_approach,
            GraspState.GRASP: self._do_grasp,
            GraspState.CLOSE_HAND: self._do_close_hand,
            GraspState.LIFT: self._do_lift,
            GraspState.PLACE: self._do_place,
            GraspState.RELEASE: self._do_release,
            GraspState.RETURN: self._do_return,
        }

        action = action_map.get(self.state)
        if action:
            action()

        return self.state not in (GraspState.IDLE, GraspState.DONE)

    # ============================================
    # 各状态动作
    # ============================================

    def _do_approach(self):
        """预接近: 移到目标正上方, 夹爪张开 (与 arm_grabber.py 第156-159行一致: mode=2)"""
        x, y, z = self.target_point
        target_z = z + self.approach_height_z
        # 预接近用较轻的下压，防止推倒瓶子 (arm_grabber.py: j2 + 0.2)
        light_down = self.down_offset + 0.2
        joints = list(self.ik_solver.solve(x, y, target_z, down_offset=light_down))
        joints[5] = self.hand_open_angle    # joint6: 夹爪张开 (1.57=全开)
        self._send(joints, mode=2)          # mode=2: 跟随模式柔和PID
        self._current_joints = joints

        self._node.get_logger().info(
            f'🔼 动作: 预接近 (目标上方 {self.approach_height_z*100:.0f}cm, IK高度={target_z:.3f}m)'
        )
        self._next_state(GraspState.GRASP)

    def _do_grasp(self):
        """下探到抓取高度, 夹爪保持张开 (与 arm_grabber.py 第161-163行一致: mode=2)"""
        x, y, z = self.target_point
        joints = list(self.ik_solver.solve(x, y, z, down_offset=self.down_offset))
        joints[5] = self.hand_open_angle    # joint6: 张开 (1.57=全开)
        self._send(joints, mode=2)          # mode=2: 跟随模式柔和PID
        self._current_joints = joints

        self._node.get_logger().info(f'🔽 动作: 下探抓取位 (目标高度={z:.3f}m)')
        self._next_state(GraspState.CLOSE_HAND)

    def _do_close_hand(self):
        """闭合夹爪 (与 arm_grabber.py 第166-167行一致: mode=1)"""
        if self._current_joints is not None:
            joints = list(self._current_joints)
            joints[5] = self.hand_close_angle   # joint6: 闭合 (-0.4)
            self._send(joints, mode=1)          # mode=1: 默认强力PID
            self._current_joints = joints

        self._node.get_logger().info('✊ 动作: 闭合夹爪')
        self._next_state(GraspState.LIFT)

    def _do_lift(self):
        """抬起物体 (与 arm_grabber.py 第170-172行一致: mode=2)"""
        x, y, z = self.target_point
        target_z = z + self.lift_height_z
        lift_down = self.down_offset + 0.7   # 下压减轻，手臂抬起
        joints = list(self.ik_solver.solve(x, y, target_z, down_offset=lift_down))
        joints[5] = self.hand_close_angle    # 保持闭合 (-0.4)
        self._send(joints, mode=2)           # mode=2: 跟随模式柔和PID
        self._current_joints = joints

        self._node.get_logger().info(
            f'⬆️ 动作: 抬起物体 (抬高 {self.lift_height_z*100:.0f}cm, IK高度={target_z:.3f}m)'
        )
        self._next_state(GraspState.DONE)

    def _do_place(self):
        """移动到放置位置 (TODO: 由用户自定义放置坐标)"""
        self._node.get_logger().info('📍 动作: 移动到放置位置')
        # 示例: 放到右侧固定位置
        place_x, place_y = 0.15, 0.10
        joints = list(self.ik_solver.solve(place_x, place_y))
        joints[5] = self.hand_close_angle
        self._send(joints)
        self._current_joints = joints
        self._next_state(GraspState.RELEASE)

    def _do_release(self):
        """张开夹爪释放物体"""
        if self._current_joints is not None:
            joints = list(self._current_joints)
            joints[5] = self.hand_open_angle
            self._send(joints)
            self._current_joints = joints

        self._node.get_logger().info('🖐️ 动作: 释放物体')
        self._next_state(GraspState.RETURN)

    def _do_return(self):
        """返回观察位姿 (复位, 与 arm_grabber.py 复位一致)"""
        home_joints = [0.0, 0.4, -0.3, -1.0, 0.0, self.hand_open_angle]
        self._send(home_joints)
        self._current_joints = home_joints

        self._node.get_logger().info('🏠 动作: 返回观察位姿')
        self._next_state(GraspState.DONE)

    # ============================================
    # 辅助方法
    # ============================================

    def _next_state(self, next_state):
        """记录时间并切换到下一个状态 (给机械臂运动留时间)"""
        self._last_step_time = time.time()
        self.state = next_state

    def cancel(self):
        """取消当前抓取序列"""
        if self.is_active:
            self._node.get_logger().warn('⚠️ 抓取序列被取消')
            self.state = GraspState.IDLE
            self.target_point = None
