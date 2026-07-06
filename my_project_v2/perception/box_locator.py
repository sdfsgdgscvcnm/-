#!/usr/bin/env python3
"""
ArUco 盒子定位 —— 检测篮子上的二维码标记 (兼容新版 OpenCV)
蓝色文具盒贴 ID=1，橙色垃圾盒贴 ID=2
"""

import cv2
import numpy as np

# ==================== ArUco 配置 ====================
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# 标记 ID 映射
MARKER_BLUE = 1
MARKER_ORANGE = 2


def find_marker(frame, marker_id):
    """
    查找指定 ID 的 ArUco 标记
    返回: (cx, cy) 或 None
    """
    corners, ids, _ = ARUCO_DETECTOR.detectMarkers(frame)

    if ids is None:
        return None

    for i, found_id in enumerate(ids.flatten()):
        if found_id == marker_id:
            corner = corners[i][0]
            cx = int(np.mean(corner[:, 0]))
            cy = int(np.mean(corner[:, 1]))
            return (cx, cy)
    return None


def draw_markers(frame):
    """在图像上画出所有检测到的标记（调试用）"""
    corners, ids, _ = ARUCO_DETECTOR.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, corner in enumerate(corners):
            cx = int(np.mean(corner[0][:, 0]))
            cy = int(np.mean(corner[0][:, 1]))
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID={ids.flatten()[i]}", (cx - 30, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return frame


# ==================== 独立测试入口 ====================
if __name__ == "__main__":
    import sys
    from hardware.camera import Camera

    cam_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cam = Camera(camera_id=cam_id)

    print("ArUco 盒子定位测试")
    print(f"  蓝色文具盒: ID={MARKER_BLUE}")
    print(f"  橙色垃圾盒: ID={MARKER_ORANGE}")
    print("  按 'q' 退出")
    print("=" * 50)

    while True:
        frame = cam.read()
        if frame is None:
            continue

        frame = draw_markers(frame)

        # 查找并打印两个标记
        for mid, name in [(MARKER_BLUE, 'Blue'), (MARKER_ORANGE, 'Orange')]:
            center = find_marker(frame, mid)
            if center:
                cv2.putText(frame, f"{name} Box", (center[0] - 40, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0) if mid == MARKER_BLUE else (0, 165, 255), 2)
                print(f"\rID={mid} ({name}): {center}", end='')

        # 画面中心十字线
        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 0, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (255, 0, 0), 1)

        cv2.imshow("Box Locator Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()
    print("程序结束")