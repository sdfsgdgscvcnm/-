#!/usr/bin/env python3
"""
决策层 —— 根据当前状态，决定下一步该执行什么任务
只负责"做什么"，不负责"怎么做"（执行由 ActionExecutor 负责）
"""


class TaskPlanner:
    def __init__(self, config):
        """
        config: 从 config.yaml 读取的参数字典
        """
        self.graspable_ids = config['graspable_ids']
        self.basket_ids = config['basket_ids']
        self.classes = config['classes']

        # 内部状态
        self.state = "init"               # 当前状态
        self.boxes_found = False          # 盒子是否已定位
        self.box_positions = {'blue': None, 'orange': None}  # 盒子坐标
        self.object_found = False         # 是否找到物体
        self.object_xy = None             # 物体坐标
        self.object_class = None          # 物体类别名
        self.object_orientation = 0       # 物体方向（0/90）
        self.servo_done = False           # 伺服是否完成
        self.pick_done = False            # 抓取是否完成
        self.place_done = False           # 放置是否完成

    def next_task(self) -> str:
        """
        根据当前状态，返回下一步要执行的任务名
        任务名: "find_boxes" / "scan" / "servo" / "pick" / "place" / "home" / "done"
        """
        # 状态机：按优先级判断
        if not self.boxes_found:
            return "find_boxes"

        if not self.object_found:
            return "scan"

        if not self.servo_done:
            return "servo"

        if not self.pick_done:
            return "pick"

        if not self.place_done:
            return "place"

        # 所有步骤完成，归位
        self._reset()
        return "home"

    # ==================== 反馈接口 ====================
    def on_boxes_found(self, blue_xy, orange_xy):
        """收到盒子定位结果"""
        self.box_positions['blue'] = blue_xy
        self.box_positions['orange'] = orange_xy
        self.boxes_found = True
        print(f"决策层：盒子已定位 蓝={blue_xy} 橙={orange_xy}")

    def on_object_found(self, xy, class_name, orientation):
        """收到物体检测结果"""
        self.object_xy = xy
        self.object_class = class_name
        self.object_orientation = orientation
        self.object_found = True
        print(f"决策层：发现物体 {class_name} 坐标={xy} 方向={orientation}°")

    def on_servo_done(self, xy):
        """伺服完成"""
        self.object_xy = xy
        self.servo_done = True
        print(f"决策层：伺服完成 坐标={xy}")

    def on_pick_done(self):
        """抓取完成"""
        self.pick_done = True
        print("决策层：抓取完成")

    def on_place_done(self):
        """放置完成"""
        self.place_done = True
        print("决策层：放置完成")

    # ==================== 查询接口 ====================
    def get_box_xy(self, class_name) -> tuple:
        if class_name in ['pen', 'eraser', 'car']:
            return self.box_positions.get('orange')   # 文具 → 橙色盒子
        else:
            return self.box_positions.get('blue')     # 纸团 → 蓝色盒子

    def get_object_info(self) -> dict:
        """返回当前物体信息"""
        return {
            'xy': self.object_xy,
            'class': self.object_class,
            'orientation': self.object_orientation
        }

    # ==================== 内部方法 ====================
    def _reset(self):
        """重置状态，准备下一轮"""
        self.object_found = False
        self.object_xy = None
        self.object_class = None
        self.object_orientation = 0
        self.servo_done = False
        self.pick_done = False
        self.place_done = False
