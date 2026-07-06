#!/usr/bin/env python3
"""
融合原厂骨架 + AI模型
骨架：apriltagSort.py 的四阶段状态机
伺服：colorTrace.py 的直接舵机微调
视觉：你的 detector.detect(frame)
盒子：AprilTag 定位
用法：
    python tasks/original_sort.py COM9 1
"""

import sys
import time
import math
import cv2

from hardware.ik_engine import Kinematics
from hardware.camera import Camera
from hardware.primitives import MotionPrimitives
from perception.detector import Detector
from perception.box_locator import find_marker

# ==================== 配置 ====================
HOME = (0.0, 150.0, 70.0)
MODEL = "best.onnx"
INPUT_SIZE = 416
CONF_THRESH = 0.3

CLASS_NAMES = ['pen', 'eraser', 'paper_trash']
GRASPABLE = {0, 1, 2}

# 盒子坐标（手动测量）
BOX_BLUE = (-70, 170)
BOX_ORANGE = (100, 170)

# 画面中心（你的摄像头分辨率）
MID_CX = 320
MID_CY = 240

# AprilTag 标记ID
MARKER_BLUE = 1
MARKER_ORANGE = 2


class OriginalSort:
    def __init__(self, arm, cam, detector, motion):
        self.arm = arm
        self.cam = cam
        self.detector = detector
        self.motion = motion

        # 原版状态变量
        self.move_x = 0.0
        self.move_y = 100.0
        self.move_z = 70.0
        self.mid_block_cx = MID_CX
        self.mid_block_cy = MID_CY
        self.mid_block_cnt = 0
        self.move_status = 0
        self.block_degress = 0
        self.cap_color_status = 0

        # 舵机微调变量（从 colorTrace 吸收）
        self.servo0 = 1500
        self.servo1 = 1500

        # 盒子坐标
        self.box_blue = BOX_BLUE
        self.box_orange = BOX_ORANGE

    def init(self):
        """初始化：归位 + 张开爪子"""
        self.motion.gripper_open()
        self.motion.rotate_wrist(0)
        self.motion.go_home()
        time.sleep(1)

        self.move_x = 0.0
        self.move_y = 100.0
        self.move_z = 70.0
        self.mid_block_cnt = 0
        self.move_status = 0
        self.block_degress = 0
        self.cap_color_status = 0
        self.servo0 = 1500
        self.servo1 = 1788

        self.motion.move_to(0, 100, 70, 1000)
        time.sleep(1)

    def run(self, cx=0, cy=0, cz=0):
        """主循环：一次完整的识别→对准→抓取→放置→归位"""
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        color_read_succed = 0

        # 1. 读摄像头 + AI推理
        frame = self.cam.read()
        if frame is None:
            return

        center, box, cls_id = self.detector.detect(frame)
        print(f"检测结果: center={center}, cls_id={cls_id}")
        if center is not None and cls_id in GRASPABLE:
            block_cx = center[0]
            block_cy = center[1]
            self.cap_color_status = cls_id


            # 方向判断
            if box is not None:
                w = box[2] - box[0]
                h = box[3] - box[1]
                self.block_degress = 90 if w > h * 1.5 else 0
            else:
                self.block_degress = 0

            color_read_succed = 1
            print(f"物体锁定！偏差 du={block_cx - self.mid_block_cx}, dv={block_cy - self.mid_block_cy}")

        # 2. 状态机（原封保留）
        if color_read_succed == 1 or self.move_status == 1:

            if self.move_status == 0:
                # ★ 第0阶段：用 colorTrace 的直接舵机微调对准物体
                du = block_cx - self.mid_block_cx
                dv = block_cy - self.mid_block_cy

                if abs(du) > 5:
                    self.servo0 += (-5.0 if du > 0 else 5.0)
                    self.servo0 = max(650, min(2400, int(self.servo0)))

                if abs(dv) > 2:
                    self.servo1 += (10.0 if dv > 0 else -10.0)
                    self.servo1 = max(500, min(2400, int(self.servo1)))

                if abs(du) <= 5 and abs(dv) <= 2:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:
                        self.mid_block_cnt = 0
                        self.move_status = 1
                else:
                    self.mid_block_cnt = 0
                    self.arm.send_str(
                        "{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}".format(
                            int(self.servo0), int(self.servo1)))
                    print(f"舵机微调: servo0={int(self.servo0)}, servo1={int(self.servo1)}")
                time.sleep(0.02)

            elif self.move_status == 1:
                # ★ 第1阶段：机械臂抓取物块（保留原版逻辑，增加爪子旋转）
                self.move_status = 2
                time.sleep(0.1)

                # 旋转爪子（横笔时转90度）
                if self.block_degress == 90:
                    self.motion.rotate_wrist(90)

                # 张开爪子
                self.arm.send_str("{#005P1000T1000!}")
                time.sleep(0.1)

                # 移动机械臂到物块上方
                l = math.sqrt(self.move_x ** 2 + self.move_y ** 2)
                sin = self.move_y / l if l > 0 else 1
                cos = self.move_x / l if l > 0 else 0
                target_x = (l + 85 + cy) * cos + cx
                target_y = (l + 85 + cy) * sin

                self.motion.move_to(target_x, target_y, 70, 1000)
                time.sleep(1.2)
                self.arm.kinematics_move(target_x, target_y, 25 + cz, 1000)
                time.sleep(1.2)

                # 抓取
                self.arm.send_str("{#005P1700T1000!}")
                time.sleep(1.2)

                # 抬起
                self.arm.kinematics_move(target_x, target_y, 120, 1000)
                time.sleep(1.2)

                # 旋转爪子归位
                self.arm.send_str("{#004P1500T1000!}")
                time.sleep(0.1)

                self.mid_block_cnt = 0

            elif self.move_status == 2:
                # ★ 第2阶段：移到盒子正上方
                # 根据类别选盒子
                if self.cap_color_status in [0, 1]:  # pen, eraser → 蓝色
                    target_xy = self.box_blue
                else:  # paper_trash → 橙色
                    target_xy = self.box_orange

                self.move_x = target_xy[0]
                self.move_y = target_xy[1]

                self.arm.kinematics_move(self.move_x, self.move_y, 120, 1000)
                time.sleep(1.2)
                self.arm.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)
                self.mid_block_cnt = 0
                self.move_status = 3

            elif self.move_status == 3:
                # ★ 第3阶段：放置 + 归位（保留原版逻辑）
                self.move_status = 0
                l = math.sqrt(self.move_x ** 2 + self.move_y ** 2)
                sin = self.move_y / l if l > 0 else 1
                cos = self.move_x / l if l > 0 else 0
                target_x = (l + 85 + cy) * cos
                target_y = (l + 85 + cy) * sin

                self.motion.move_to(target_x, target_y, 70, 1000)
                time.sleep(1.2)
                self.arm.kinematics_move(target_x, target_y, 25 + cz, 1000)
                time.sleep(1.2)

                # 松开爪子
                self.arm.send_str("{#005P1000T1000!}")
                time.sleep(1.2)

                self.motion.move_to(target_x, target_y, 70, 1000)
                time.sleep(1.2)

                # 归位
                self.move_x = 0.0
                self.move_y = 100.0
                self.arm.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)

                self.mid_block_cnt = 0
                self.cap_color_status = 0


# ==================== 主程序入口 ====================
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM9'
    cam_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    arm = Kinematics(port)
    cam = Camera(camera_id=cam_id)
    detector = Detector(MODEL, input_size=INPUT_SIZE, conf_thresh=CONF_THRESH,
                        offset_x=53, offset_y=100)
    motion = MotionPrimitives(arm, home_xyz=HOME)

    app = OriginalSort(arm, cam, detector, motion)
    app.init()

    print("=" * 50)
    print("融合原厂骨架 + AI模型")
    print("按 Enter 执行一次抓取放置，输入 q 退出")
    print("=" * 50)

    while True:
        cmd = input("\n>>> ").strip()
        if cmd == 'q':
            break
        app.run(0, 0, 0)

    arm.close()
    cam.release()
    print("程序结束")


if __name__ == "__main__":
    main()