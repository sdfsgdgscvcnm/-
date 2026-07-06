#!/usr/bin/env python3
"""
摄像头模块 —— 完全独立
提供：打开摄像头、读取帧、显示画面、释放
用法：
    from camera import Camera
    cam = Camera(camera_id=1, width=640, height=480)
    frame = cam.read()
    cam.show(frame)      # 显示画面
    cam.release()

独立测试：
    python camera.py [摄像头ID]
    按 'q' 退出
"""

import cv2
import sys


class Camera:
    def __init__(self, camera_id=1, width=640, height=480):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.cap = None
        self.open()

    def open(self):
        """打开摄像头并设置分辨率"""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 {self.camera_id}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def read(self):
        # 清空缓冲，只取最新帧（K1增加次数以消除USB延迟）
        for _ in range(5):
            self.cap.grab()
        ret, frame = self.cap.retrieve()
        return frame if ret else None

    def show(self, frame, window_name="Camera"):
        """显示画面，按 'q' 退出"""
        cv2.imshow(window_name, frame)
        return cv2.waitKey(1) & 0xFF

    def release(self):
        """释放摄像头"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


# ==================== 独立测试入口 ====================
if __name__ == "__main__":
    # 默认摄像头 ID：笔记本内置通常为 0，USB 外置通常为 1
    camera_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print(f"打开摄像头 {camera_id}，按 'q' 退出...")
    cam = Camera(camera_id=camera_id)
    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("读取帧失败")
                break
            cv2.imshow("Camera Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
        print("摄像头已关闭")
