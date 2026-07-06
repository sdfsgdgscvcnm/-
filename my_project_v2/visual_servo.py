#!/usr/bin/env python3
"""
实时视觉伺服（修复版）
主线程直接控制，冷却期防指令堆积，画面流畅不瞎转
"""

import sys
import time
import cv2

from hardware.ik_engine import Kinematics
from hardware.camera import Camera
from perception.detector import Detector as PenDetector

# ==================== 工作参数 ====================
HOME_X, HOME_Y, HOME_Z = 0.0, 150.0, 70.0
GRASP_Z = 10.0
GRASP_Z_DOWN = 6.0
LIFT_Z = 120.0
X_MIN, X_MAX = -200.0, 200.0
Y_MIN, Y_MAX = 0.0, 300.0
PEN_Y_RETRACT = -30
GAIN_X = 0.04
GAIN_Y = 0.04
DEAD_ZONE = 30
CONF_THRESH = 0.5
MODEL_PATH = "best_k1.q.onnx"
# 可抓取的类别ID
GRASPABLE = {0, 1, 2}

# 冷却期：两次移动之间至少间隔 500ms
COOLDOWN_MS = 50
ALIGN_COUNT_NEEDED = 3


def main():
    if sys.platform.startswith('win'):
        default_port = 'COM9'
        default_cam = 1
    else:
        default_port = '/dev/ttyUSB0'
        default_cam = 20

    port = sys.argv[1] if len(sys.argv) > 1 else default_port
    cam_id = int(sys.argv[2]) if len(sys.argv) > 2 else default_cam

    print(f"连接机械臂: {port}")
    arm = Kinematics(port)
    print(f"打开摄像头: {cam_id}")
    cam = Camera(camera_id=cam_id)
    print(f"加载模型: {MODEL_PATH}")
    detector = PenDetector(MODEL_PATH, input_size=320)

    # 归位
    arm.send_str("{#005P1000T500!}")
    time.sleep(0.5)
    arm.kinematics_move(HOME_X, HOME_Y, HOME_Z, 1000)
    time.sleep(1)

    cur_x, cur_y, cur_z = HOME_X, HOME_Y, HOME_Z
    global GAIN_X, GAIN_Y, CONF_THRESH
    align_counter = 0
    last_move_time = 0   # 上次移动的时间戳

    print("=" * 55)
    print("实时视觉伺服")
    print(f"  增益: GAIN_X={GAIN_X}, GAIN_Y={GAIN_Y}")
    print(f"  死区: {DEAD_ZONE} px  冷却: {COOLDOWN_MS}ms")
    print("  'q'退出 | 'g'抓取 | 'r'归位")
    print("=" * 55)

    locked_cls = None
    locked_center = None
    locked_time = 0          # ★ 锁定时间戳
    align_counter = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.01)
                continue
                        # ★ 先检测 AprilTag 标记位置
            from perception.box_locator import find_marker
            marker_blue = find_marker(frame, 1)   # 蓝色盒子标记ID=1
            marker_orange = find_marker(frame, 2)  # 橙色盒子标记ID=2

            # 检测物体
            center, _, cls_id = detector.detect(frame)

            # ★ 如果检测结果在 AprilTag 标记附近（100像素内），忽略
            if center is not None:
                for marker_pos in [marker_blue, marker_orange]:
                    if marker_pos is not None:
                        dist = ((center[0] - marker_pos[0])**2 + (center[1] - marker_pos[1])**2)**0.5
                        if dist < 300:   # 在标记100像素范围内
                            center = None   # 视为篮子误检，忽略
                            break
            
            mid_x, mid_y = detector.mid_x, detector.mid_y

            # 2. 画十字线
            cv2.line(frame, (mid_x, 0), (mid_x, frame.shape[0]), (255, 0, 0), 1)
            cv2.line(frame, (0, mid_y), (frame.shape[1], mid_y), (255, 0, 0), 1)

            # 3. 如果发现目标，果断锁定，不再理会其他目标
            if center is not None and cls_id in GRASPABLE:
                # 锁定目标后，不再更新目标位置，直到抓取完成
                cx, cy = center
                du, dv = detector.get_deviation(cx, cy)
                    
               # 画笔和偏差线
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), 2)
                cv2.line(frame, (mid_x, mid_y), (cx, cy), (0, 0, 255), 2)
                cv2.putText(frame, f"du={du:+d} dv={dv:+d}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # 伺服控制
                now = time.time()
                if abs(du) > DEAD_ZONE or abs(dv) > DEAD_ZONE:
                    align_counter = 0
                    if (now - last_move_time) * 1000 >= COOLDOWN_MS:
                        x_step = du * GAIN_X
                        y_step = dv * GAIN_Y
                        max_step = 5.0
                        x_step = max(-max_step, min(max_step, x_step))
                        y_step = max(-max_step, min(max_step, y_step))

                        target_x = cur_x + x_step
                        target_y = cur_y - y_step
                        target_x = max(X_MIN, min(X_MAX, target_x))
                        target_y = max(Y_MIN, min(Y_MAX, target_y))

                        if locked_cls == 0:
                            target_y += PEN_Y_RETRACT 

                        if ((target_x - cur_x)**2 + (target_y - cur_y)**2)**0.5 > 0.5:
                            arm.kinematics_move(target_x, target_y, cur_z, 200)
                            cur_x, cur_y = target_x, target_y
                            last_move_time = now
                else:
                    align_counter += 1
                    if align_counter >= ALIGN_COUNT_NEEDED:
                        cv2.putText(frame, "ALIGNED - Press 'g'", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(frame, f"aligning... {align_counter}/{ALIGN_COUNT_NEEDED}", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                align_counter = 0
                cv2.putText(frame, "No object", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(frame, f"Gx={GAIN_X:.2f} Gy={GAIN_Y:.2f}",
                        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow("Visual Servo", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                if center and align_counter >= ALIGN_COUNT_NEEDED:
                    print("抓取！")
                    grasp_x = cur_x
                    grasp_y = cur_y
                    if cls_id == 0:   # 笔
                        grasp_y += 25   # 往前伸25mm
                    grasp_y = min(Y_MAX, grasp_y)
                    # 1. 下降抓取
                    arm.kinematics_move(grasp_x, grasp_y, GRASP_Z, 800)
                    time.sleep(1)
                    arm.send_str("{#005P1700T500!}")
                    time.sleep(0.5)
                    # 2. 二次下压
                    arm.kinematics_move(grasp_x, grasp_y, GRASP_Z_DOWN, 400)
                    time.sleep(0.5)
                    # 3. 垂直抬升（从抓取位出发，不回缩）
                    arm.kinematics_move(grasp_x, grasp_y, 120, 800)
                    time.sleep(1)
                    # 4. 水平移到释放点上方
                    arm.kinematics_move(HOME_X, HOME_Y, 120, 1000)
                    time.sleep(1)
                    # 5. 垂直下降到释放高度
                    arm.kinematics_move(HOME_X, HOME_Y, 25, 600)
                    time.sleep(0.8)
                    # 6. 释放
                    arm.send_str("{#005P1000T500!}")
                    time.sleep(0.5)
                    # 7. 归位
                    arm.kinematics_move(HOME_X, HOME_Y, HOME_Z, 1000)
                    time.sleep(1)

                    cur_x, cur_y = HOME_X, HOME_Y
                    last_move_time = time.time()
                    align_counter = 0
                else:
                    print("未对准，无法抓取")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        cam.release()
        arm.close()
        cv2.destroyAllWindows()
        print("程序结束")


if __name__ == "__main__":
    main()
