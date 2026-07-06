#!/usr/bin/env python3
"""
执行层 —— 接收任务指令，调用底层模块执行
只负责"怎么做"，不负责"做什么"（决策由 TaskPlanner 负责）
"""

import time
import math


class ActionExecutor:
    def __init__(self, config, motion, camera, detector, box_locator):
        """
        motion: MotionPrimitives 实例
        camera: Camera 实例
        detector: Detector 实例
        box_locator: BoxLocator 模块（find_marker 函数）
        """
                # 扫描参数
        self.scan_x = config['scan_x']
        self.scan_y = config['scan_y']
        self.scan_step = config['scan_step']
        self.scan_z = config['scan_z']
        self.scan_speed = config['scan_speed']
        self.scan_settle = config['scan_settle']

        # 伺服参数
        self.gain_x = config['gain_x']
        self.gain_y = config['gain_y']
        self.dead_zone = config['dead_zone']
        self.cooldown_ms = config['cooldown_ms']
        self.align_count = config['align_count']
        self.max_move_step = config.get('max_move_step', 8.0)
        self.x_min = config.get('x_min', -150.0)
        self.x_max = config.get('x_max', 150.0)
        self.y_min = config.get('y_min', 0.0)
        self.y_max = config.get('y_max', 300.0)

        # 抓取参数
        self.grasp_z = config['grasp_z']
        self.grasp_z_down = config['grasp_z_down']
        self.lift_z = config['lift_z']
        self.release_z = config['release_z']
        self.home_xyz = tuple(config['home'])

        # 模型和类别
        self.graspable_ids = config['graspable_ids']
        self.basket_ids = config['basket_ids']
        self.classes = config['classes']

        # 盒子定位
        self.marker_blue = config['marker_blue']
        self.marker_orange = config['marker_orange']

        # ★ 保存传入的底层模块
        self.motion = motion
        self.camera = camera
        self.detector = detector
        self.box_locator = box_locator
        # 抓取补偿
        self.grasp_compensations = config.get('grasp_compensations', {0: 25, 1: 0, 2: 0})

        # 锁定策略
        self.lock_strategy = config.get('lock_strategy', 'nearest')
        self.lock_distance_threshold = config.get('lock_distance_threshold', 60)

        # 盒子定位策略
        self.box_select_strategy = config.get('box_select_strategy', 'min_deviation')

        # 抓取确认
        self.grasp_verify_enabled = config.get('grasp_verify_enabled', False)

    # ==================== 扫描找物体 ====================
    def scan(self) -> dict:
        """
        蛇形扫描桌面，寻找可抓取物体
        返回: {'found': True/False, 'x': float, 'y': float, 'class': str, 'orientation': int}
        """
        x_min, x_max = self.scan_x
        y_min, y_max = self.scan_y
        total_points = 0

        y_mid = (int(y_max) + int(y_min)) // 2
        y_far = list(range(y_mid, int(y_max) + int(self.scan_step), int(self.scan_step)))
        y_near = list(range(y_mid - int(self.scan_step), int(y_min) - int(self.scan_step), -int(self.scan_step)))
        y_values = y_far + y_near

        for row, y in enumerate(y_values):
            if row % 2 == 0:
                x_values = list(range(int(x_min), int(x_max) + int(self.scan_step), int(self.scan_step)))
            else:
                x_values = list(range(int(x_max), int(x_min) - int(self.scan_step), -int(self.scan_step)))

            for x in x_values:
                total_points += 1
                self.motion.move_to(x, y, self.scan_z, self.scan_speed)
                time.sleep(self.scan_settle / 1000.0)

                frame = self.camera.read()
                if frame is not None:
                    center, box, cls_id = self.detector.detect(frame)
                    if center is not None and cls_id in self.graspable_ids:
                        class_name = self.classes[cls_id] if cls_id < len(self.classes) else 'unknown'
                        orientation = self.detector.get_orientation(box, cls_id)
                        print(f"\n找到 {class_name}！坐标: ({x}, {y})")
                        return {
                            'found': True,
                            'x': x, 'y': y,
                            'class': class_name,
                            'orientation': orientation
                        }

                print(f"\r扫描中... 第 {total_points} 个点 ({x}, {y})", end='')

        print(f"\n扫描完成，共检查 {total_points} 个点，未找到物体")
        return {'found': False}

    # ==================== 视觉伺服 ====================
    def servo(self, start_xy: tuple) -> tuple:
        cur_x, cur_y = start_xy
        cur_z = self.scan_z
        last_move_time = 0
        align_counter = 0
        locked_cls = None

        servo_dead_zone = self.dead_zone
        servo_align_count = self.align_count

        while True:
            for _ in range(2):
                self.camera.cap.grab()
            ret, frame = self.camera.cap.retrieve()
            if not ret or frame is None:
                continue

            center, box, cls_id = self.detector.detect(frame)

            if center is not None and cls_id in self.graspable_ids:
                if locked_cls is None:
                    locked_cls = cls_id
                # ★ 根据配置的锁定策略决定是否过滤
                if self.lock_strategy == "nearest":
                    # 只追踪离上次位置最近的物体（后续会加距离判断）
                    if cls_id != locked_cls:
                        center = None
                elif self.lock_strategy == "highest_conf":
                    # 不锁定类别，每帧选置信度最高的（默认行为）
                    pass
                elif self.lock_strategy == "largest_y":
                    # 锁定第一个物体的类别
                    if cls_id != locked_cls:
                        center = None

            if center is None:
                align_counter = 0
                continue

            du, dv = self.detector.get_deviation(center[0], center[1])

            if abs(du) <= servo_dead_zone and abs(dv) <= servo_dead_zone:
                align_counter += 1
                if align_counter >= servo_align_count:
                    return (cur_x, cur_y)
            else:
                align_counter = 0
                now = time.time()
                if (now - last_move_time) * 1000 >= self.cooldown_ms:
                    x_step = du * self.gain_x
                    y_step = dv * self.gain_y
                    max_step = self.max_move_step if hasattr(self, 'max_move_step') else 8.0
                    x_step = max(-max_step, min(max_step, x_step))
                    y_step = max(-max_step, min(max_step, y_step))

                    target_x = cur_x + x_step
                    target_y = cur_y - y_step
                    target_x = max(self.x_min, min(self.x_max, target_x))
                    target_y = max(self.y_min, min(self.y_max, target_y))

                    self.motion.move_to(target_x, target_y, cur_z, 200)
                    cur_x, cur_y = target_x, target_y
                    last_move_time = now

    # ==================== 抓取 ====================
    def pick(self, xy: tuple, orientation: int = 0, cls_id: int = -1):
        x, y = xy
        comp = self.grasp_compensations.get(cls_id, 0)
        if comp != 0:
            y += comp
            y = min(self.y_max, y)
        # 抬升到安全高度
        self.motion.move_to(x, y, self.lift_z, 600)
        # 旋转爪子
        if orientation != 0:
            self.motion.rotate_wrist(orientation)
        # 下降抓取
        self.motion.move_to(x, y, self.grasp_z, 800)
        self.motion.gripper_close()
        self.motion.move_to(x, y, self.grasp_z_down, 400)
        self.motion.move_to(x, y, self.lift_z, 800)
        print(f"抓取完成！")

    # ==================== 放置 ====================
    def place(self, xy: tuple):
        """
        在指定坐标上方释放物体（不下压）
        xy: (x, y) 盒子坐标
        """
        x, y = xy
        # 直接移到盒子正上方
        self.motion.move_to(x, y, self.lift_z, 800)
        # 张开爪子，物体自由落体
        self.motion.gripper_open()
        time.sleep(0.5)
        # 抬升离开
        self.motion.move_to(x, y, self.lift_z + 40, 400)
        print(f"放置完成！")

    # ==================== 归位 ====================
    def go_home(self):
        self.motion.rotate_wrist(0)
        self.motion.go_home()
        print("归位完成")

    # ==================== 找盒子（极坐标旋转 + ArUco） ====================
    def find_boxes(self) -> dict:
        """
        固定扫描点 + 角度补偿 + 全扫描后选偏差最小的坐标
        """
        boxes = {'blue': None, 'orange': None}
        # 存储候选: (box_x, box_y, pixel_error)
        candidates = {'blue': [], 'orange': []}

        scan_points = [
            (100, 0, 120, 90),
            (87, 50, 120, 60),
            (71, 71, 120, 45),
            (50, 87, 120, 30),
            (0, 100, 120, 0),
            (-50, 87, 120, -30),
            (-71, 71, 120, -45),
            (-87, 50, 120, -60),
            (-100, 0, 120, -90)
        ]

        forward_dist = 50.0

        print("找盒子中（全扫描，选偏差最小坐标）...")
        for (x, y, z, angle_deg) in scan_points:
            self.motion.move_to(x, y, z, 400)
            for _ in range(2):
                self.camera.read()
            time.sleep(1.5)

            for _ in range(5):
                self.camera.cap.grab()
            ret, frame = self.camera.cap.retrieve()
            if not ret or frame is None:
                continue

            for color, marker_id in [('blue', self.marker_blue), ('orange', self.marker_orange)]:
                center = self.box_locator(frame, marker_id)
                if center is not None:
                    base_angle = math.radians(angle_deg)
                    x_comp = forward_dist * math.sin(base_angle)
                    y_comp = forward_dist * math.cos(base_angle)

                    box_x = x + x_comp
                    box_y = y + y_comp

                    # ★ 计算标记在画面中的像素偏差
                    du = center[0] - (frame.shape[1] // 2)
                    dv = center[1] - (frame.shape[0] // 2)
                    pixel_error = math.sqrt(du**2 + dv**2)  # 偏差越小越准

                    candidates[color].append((box_x, box_y, pixel_error))
                    print(f"    角度{angle_deg}°发现{color}盒子候选: ({box_x:.0f}, {box_y:.0f}) 偏差={pixel_error:.0f}px")

            # ★ 根据配置的策略选择最优候选
        for color in ['blue', 'orange']:
            if candidates[color]:
                if self.box_select_strategy == "min_deviation":
                    candidates[color].sort(key=lambda item: item[2])  # 偏差升序
                elif self.box_select_strategy == "max_angle":
                    candidates[color].sort(key=lambda item: item[2], reverse=True)  # 偏差降序（角度大对应偏差可能大）
                best = candidates[color][0]
                boxes[color] = (best[0], best[1])
                print(f"  ✅ {color}色盒子最终坐标: ({best[0]:.0f}, {best[1]:.0f}) (偏差{best[2]:.0f}px)")
            else:
                print(f"  ⚠️ 未找到{color}色盒子")

        self.motion.go_home()
        time.sleep(0.5)
        return boxes
