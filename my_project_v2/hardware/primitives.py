#!/usr/bin/env python3
"""
原子动作库 —— 最基础的机械臂动作单元
所有上层模块最终都通过这里控制硬件
"""

import time
import math


class MotionPrimitives:
    def __init__(self, arm, home_xyz=(0.0, 100.0, 120.0)):
        """
        arm: Kinematics 实例（来自 ik_engine.py）
        home_xyz: 归位坐标
        """
        self.arm = arm
        self.home = home_xyz
        self.pos = home_xyz
        self._current_base_angle = 0.0   # 底座当前角度（防回弹）

    # ==================== 爪子控制 ====================
    def gripper_open(self):
        self.arm.send_str("{#005P1000T500!}")
        time.sleep(0.5)

    def gripper_close(self):
        self.arm.send_str("{#005P1700T500!}")
        time.sleep(0.5)
    
    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, speed_ms=800):
        """
        相对当前位置移动
        dx, dy, dz: 各轴偏移量 (mm)
        speed_ms: 移动耗时 (毫秒)
        """
        new_x = self.pos[0] + dx
        new_y = self.pos[1] + dy
        new_z = self.pos[2] + dz
        self.move_to(new_x, new_y, new_z, speed_ms)

    # ==================== 移动控制 ====================
    def move_to(self, x, y, z, speed_ms=800):
        """绝对坐标移动，直接调用逆运动学"""
        self.arm.kinematics_move(x, y, z, speed_ms)
        self.pos = (x, y, z)
        time.sleep(speed_ms / 1000.0 + 0.2)

    def go_home(self, speed_ms=1000):
        """归位到预设观察位"""
        self.move_to(*self.home, speed_ms)

    def get_position(self):
        """返回当前位置 (x, y, z)"""
        return self.pos

    # ==================== 底座旋转（防回弹） ====================
    def rotate_base(self, target_angle, total_time_ms=600, min_step_ms=15):
        """平滑旋转底座，并自动更新末端坐标"""
        cur_angle = self._current_base_angle
        d_angle = target_angle - cur_angle
        steps = max(40, total_time_ms // min_step_ms)

        for i in range(1, steps + 1):
            progress = i / steps
            smooth = progress - math.sin(2 * math.pi * progress) / (2 * math.pi)
            inter_angle = cur_angle + d_angle * smooth
            pwm = int(1500 + 2000.0 * inter_angle / 270.0)
            pwm = max(1000, min(2000, pwm))
            self.arm.send_str(f"{{#000P{pwm:0>4d}T{min_step_ms:0>4d}!}}")
            time.sleep(min_step_ms / 1000.0 + 0.01)

        # ★ 更新底座角度（先保存旧值）
        old_angle = self._current_base_angle
        self._current_base_angle = target_angle

        # ★ 根据旋转角度更新末端位置
        cur_x, cur_y, cur_z = self.pos
        R = math.sqrt(cur_x**2 + cur_y**2)
        phi = math.atan2(cur_x, cur_y)
        new_phi = phi + math.radians(target_angle - old_angle)  # ★ 用旧角度算差值
        new_x = -R * math.sin(new_phi)
        new_y = R * math.cos(new_phi)
        self.pos = (new_x, new_y, cur_z)

    # ==================== 爪子旋转 ====================
    def rotate_wrist(self, angle_deg):
        """
        旋转爪子（4号舵机）
        angle_deg > 0 → 顺时针
        """
        angle_deg = -angle_deg
        pwm_change = int(angle_deg * 7.67)
        pwm = 1537 + pwm_change
        pwm = max(500, min(2500, pwm))
        self.arm.send_str(f"{{#004P{pwm:0>4d}T500!}}")
        time.sleep(0.5)

    def wait(self, ms):
        time.sleep(ms / 1000.0)