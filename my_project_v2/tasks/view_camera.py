#!/usr/bin/env python3
"""
实时显示摄像头画面 + 检测结果
用法：
    python tasks/view_camera.py 1
"""

import sys
import cv2
from hardware.camera import Camera
from perception.detector import Detector

MODEL = "best.onnx"
INPUT_SIZE = 416
CONF_THRESH = 0.3
CLASS_NAMES = ['pen', 'eraser', 'paper_trash']


def main():
    cam_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1

    cam = Camera(camera_id=cam_id)
    detector = Detector(MODEL, input_size=INPUT_SIZE, conf_thresh=CONF_THRESH,
                        offset_x=53, offset_y=100)

    print("实时显示检测结果，按 'q' 退出")
    
    while True:
        frame = cam.read()
        if frame is None:
            continue

        h, w = frame.shape[:2]
        mid_x, mid_y = w // 2, h // 2

        # 画十字线
        cv2.line(frame, (mid_x, 0), (mid_x, h), (255, 0, 0), 1)
        cv2.line(frame, (0, mid_y), (w, mid_y), (255, 0, 0), 1)

        # 检测物体
        center, box, cls_id = detector.detect(frame)

        if center and box:
            cx, cy = center
            du, dv = detector.get_deviation(cx, cy)
            x1, y1, x2, y2 = box

            # 画框
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(frame, (mid_x, mid_y), (cx, cy), (0, 0, 255), 2)

            # 显示信息
            class_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else '?'
            cv2.putText(frame, f"{class_name} du={du:+d} dv={dv:+d}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            print(f"\r{class_name}: ({cx},{cy}) 偏差 du={du:+d} dv={dv:+d}", end='')
        else:
            cv2.putText(frame, "No object", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            print(f"\r未检测到物体", end='')

        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()
    print("\n程序结束")


if __name__ == "__main__":
    main()