#!/usr/bin/env python3
"""
视觉伺服引擎 —— 只负责根据像素偏差微调机械臂
用法：
    from actions.visual_servo import VisualServo
    servo = VisualServo(config, motion, cam, detector)
    final_xy = servo.run(start_xy)
"""

import time

class VisualServo:
    def __init__(self, config, motion, cam, detector):
        self.motion = motion
        self.cam = cam
        self.detector = detector

        # 从 config 读取参数
        self.gain_x = config['gain_x']
        self.gain_y = config['gain_y']
        self.dead_zone = config['dead_zone']
        self.cooldown_ms = config['cooldown_ms']
        self.align_count = config['align_count']
        self.max_move_step = config['max_move_step']
        self.x_min = config['x_min']
        self.x_max = config['x_max']
        self.y_min = config['y_min']
        self.y_max = config['y_max']

    def run(self, start_xy: tuple) -> tuple:
        """
        从 start_xy 开始视觉伺服精调
        返回: (final_x, final_y) 对准后的坐标，失败返回 None
        """
        cur_x, cur_y = start_xy
        cur_z = self.motion.get_position()[2]
        last_move_time = 0
        align_counter = 0

        print("伺服精调中...")
        while True:
            frame = self.cam.read()
            if frame is None:
                continue

            center, _, _ = self.detector.detect(frame)
            if center is None:
                align_counter = 0
                continue

            du, dv = self.detector.get_deviation(center[0], center[1])

            if abs(du) <= self.dead_zone and abs(dv) <= self.dead_zone:
                align_counter += 1
                if align_counter >= self.align_count:
                    print(f"伺服完成！坐标: ({cur_x:.0f}, {cur_y:.0f})")
                    return (cur_x, cur_y)
            else:
                align_counter = 0
                now = time.time()
                if (now - last_move_time) * 1000 >= self.cooldown_ms:
                    # 步幅和偏差成正比，限制单次最大移动量
                    x_step = du * self.gain_x
                    y_step = dv * self.gain_y
                    x_step = max(-self.max_move_step, min(self.max_move_step, x_step))
                    y_step = max(-self.max_move_step, min(self.max_move_step, y_step))

                    target_x = cur_x + x_step
                    target_y = cur_y - y_step
                    target_x = max(self.x_min, min(self.x_max, target_x))
                    target_y = max(self.y_min, min(self.y_max, target_y))

                    self.motion.move_to(target_x, target_y, cur_z, 200)
                    cur_x, cur_y = target_x, target_y
                    last_move_time = now