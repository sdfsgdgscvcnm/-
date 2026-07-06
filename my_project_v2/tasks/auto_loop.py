#!/usr/bin/env python3
"""
自动循环抓取测试
持续检测物体，自动对准、抓取、放置、归位
按 Ctrl+C 停止
用法：
    python tasks/auto_loop.py COM9 1
"""

import sys
import time
import math
import cv2

from hardware.ik_engine import Kinematics
from hardware.camera import Camera
from hardware.primitives import MotionPrimitives
from perception.detector import Detector

HOME = (0.0, 150.0, 70.0)
MODEL = "best.onnx"
INPUT_SIZE = 416
CONF_THRESH = 0.3
CLASS_NAMES = ['pen', 'eraser', 'paper_trash']
GRASPABLE = {0, 1, 2}

MID_CX = 320
MID_CY = 240

BOX_BLUE = (-70, 170)
BOX_ORANGE = (100, 170)


class AutoLoop:
    def __init__(self, arm, cam, detector, motion):
        self.arm = arm
        self.cam = cam
        self.detector = detector
        self.motion = motion

        # 原版状态变量
        self.move_x = 0.0
        self.move_y = 100.0
        self.move_z = 70.0
        self.mid_block_cnt = 0
        self.move_status = 0
        self.block_degress = 0
        self.cap_color_status = 0

        # 舵机微调变量
        self.servo0 = 1500
        self.servo1 = 1788

    def init(self):
        self.motion.gripper_open()
        self.motion.rotate_wrist(0)
        self.motion.go_home()
        time.sleep(1)

        self.move_x = 0.0
        self.move_y = 100.0
        self.move_z = 70.0
        self.mid_block_cnt = 0
        self.move_status = 0
        self.cap_color_status = 0
        self.servo0 = 1500
        self.servo1 = 1788

    def step(self):
        """执行一次循环（读取一帧 + 状态机判断 + 执行动作）"""
        block_cx = MID_CX
        block_cy = MID_CY
        color_read_succed = 0

        # 读取摄像头
        frame = self.cam.read()
        if frame is None:
            return

        # AI检测
        center, box, cls_id = self.detector.detect(frame)

        if center is not None and cls_id in GRASPABLE:
            block_cx = center[0]
            block_cy = center[1]
            self.cap_color_status = cls_id

            if box is not None:
                w = box[2] - box[0]
                h = box[3] - box[1]
                self.block_degress = 90 if w > h * 1.5 else 0
            else:
                self.block_degress = 0

            color_read_succed = 1

        # 状态机
        if color_read_succed == 1 or self.move_status in [1, 2]:

            if self.move_status == 0:
                # 对准：直接舵机微调
                du = block_cx - MID_CX
                dv = block_cy - MID_CY

                if abs(du) > 5:
                    self.servo0 += (-10.0 if du > 0 else 10.0)
                    self.servo0 = max(650, min(2400, int(self.servo0)))
                if abs(dv) > 2:
                    self.servo1 += (-10.0 if dv > 0 else 10.0)
                    self.servo1 = max(500, min(2400, int(self.servo1)))

                if abs(du) <= 5 and abs(dv) <= 2:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:
                        self.mid_block_cnt = 0
                        self.move_status = 1
                        print("已对准，开始抓取！")
                else:
                    self.mid_block_cnt = 0
                    self.arm.send_str(
                        "{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}".format(
                            int(self.servo0), int(self.servo1)))

            elif self.move_status == 1:
                # 抓取
                self.move_status = 2
                time.sleep(0.1)

                if self.block_degress == 90:
                    self.motion.rotate_wrist(90)

                self.arm.send_str("{#005P1000T1000!}")
                time.sleep(0.1)

                l = math.sqrt(self.move_x**2 + self.move_y**2)
                sin = self.move_y / l if l > 0 else 1
                cos = self.move_x / l if l > 0 else 0
                target_x = (l + 85) * cos
                target_y = (l + 85) * sin

                self.arm.kinematics_move(target_x, target_y, 70, 1000)
                time.sleep(1.2)
                self.arm.kinematics_move(target_x, target_y, 25, 1000)
                time.sleep(1.2)
                self.arm.send_str("{#005P1700T1000!}")
                time.sleep(1.2)
                self.arm.kinematics_move(target_x, target_y, 120, 1000)
                time.sleep(1.2)
                self.arm.send_str("{#004P1500T1000!}")
                time.sleep(0.1)

                self.mid_block_cnt = 0
                print("抓取完成，移到盒子...")

            elif self.move_status == 2:
                # 放置
                target_xy = BOX_BLUE if self.cap_color_status in [0, 1] else BOX_ORANGE
                self.move_x = target_xy[0]
                self.move_y = target_xy[1]

                self.arm.kinematics_move(self.move_x, self.move_y, 120, 1000)
                time.sleep(1.2)
                self.arm.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)
                self.move_status = 3

            elif self.move_status == 3:
                # 归位
                self.move_status = 0
                self.mid_block_cnt = 0
                self.cap_color_status = 0
                self.servo0 = 1500
                self.servo1 = 1788

                self.arm.kinematics_move(self.move_x, self.move_y, 25, 1000)
                time.sleep(1.2)
                self.arm.send_str("{#005P1000T1000!}")
                time.sleep(1.2)
                self.arm.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)

                self.move_x = 0.0
                self.move_y = 100.0
                self.arm.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)
                print("归位完成，继续检测...\n")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM9'
    cam_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    arm = Kinematics(port)
    cam = Camera(camera_id=cam_id)
    detector = Detector(MODEL, input_size=INPUT_SIZE, conf_thresh=CONF_THRESH,
                        offset_x=53, offset_y=100)
    motion = MotionPrimitives(arm, home_xyz=HOME)

    app = AutoLoop(arm, cam, detector, motion)
    app.init()

    print("=" * 50)
    print("自动循环抓取测试")
    print("按 Ctrl+C 停止")
    print("=" * 50)

    try:
        while True:
            app.step()
    except KeyboardInterrupt:
        print("\n程序结束")
    finally:
        arm.close()
        cam.release()


if __name__ == "__main__":
    main()