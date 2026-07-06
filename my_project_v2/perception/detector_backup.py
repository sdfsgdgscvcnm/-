#!/usr/bin/env python3
"""
物体检测器 —— 模型推理 + 方向判断 + 偏差计算
提供统一的 detect() 接口，返回物体信息
"""
import time
import cv2
import numpy as np

import onnxruntime as ort
import sys
# ★ 无条件导入 spacemit_ort（它必须在创建 InferenceSession 前被导入）
try:
    import spacemit_ort
except ImportError:
    pass   # 笔记本上可能没有，忽略


class Detector:
    def __init__(self, model_path, input_size=416, conf_thresh=0.3,
                 offset_x=53, offset_y=250, pen_y_offset=100, providers=None):
        """
        model_path: 量化模型路径
        input_size: 模型输入尺寸
        conf_thresh: 置信度阈值
        offset_x, offset_y: 摄像头物理偏移补偿
        pen_y_offset: 笔的额外Y轴补偿（像素）
        """
        self.class_priority = {0: 1, 1: 2, 2: 3} 
        self.input_size = input_size
        self.conf_thresh = conf_thresh
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.pen_y_offset = pen_y_offset
        self.mid_x = 320
        self.mid_y = 240
          # 锁定时间戳

        # 自动选择推理后端
        if providers is None:
            if sys.platform.startswith('win'):
                providers = ['CPUExecutionProvider']
            else:
                import spacemit_ort
                providers = ['SpaceMITExecutionProvider', 'CPUExecutionProvider']

        self.session = ort.InferenceSession(model_path, providers=[
            'SpaceMITExecutionProvider',
            'CPUExecutionProvider'
        ])
        self.input_name = self.session.get_inputs()[0].name
    def detect(self, frame):
        h, w = frame.shape[:2]
        self.mid_x = w // 2
        self.mid_y = h // 2

        # 预处理
        img = cv2.cvtColor(cv2.resize(frame, (self.input_size, self.input_size)), cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)

        # 推理
        outputs = self.session.run(None, {self.input_name: img})
        preds = np.squeeze(outputs[0]).transpose(1, 0)

        candidates = []  # ★ 收集所有候选目标


        for pred in preds:
            cls_confs = pred[4:9]
            cls_id = np.argmax(cls_confs)
            conf = cls_confs[cls_id]
            if conf < self.conf_thresh:
                continue

            cx = int(pred[0] * w / self.input_size)
            cy = int(pred[1] * h / self.input_size)
            bw_temp = int(pred[2] * w / self.input_size)
            bh_temp = int(pred[3] * h / self.input_size)
            area = bw_temp * bh_temp

            # 过滤篮子误检
            if area > w * h * 0.15:
                continue

            candidates.append((cx, cy, cls_id, conf, bw_temp, bh_temp))

        if not candidates:
            return None, None, None

        # 按类别优先级排序
        if self.class_priority:
            candidates.sort(key=lambda c: self.class_priority.get(c[2], 99))

        # 选优先级最高且离画面中心最近的物体
        best_dist = float('inf')
        best_center, best_box, best_cls = None, None, 0
        for cand in candidates:
            cx, cy, cls_id, conf, bw, bh = cand
            dist_to_center = (cx - w//2)**2 + (cy - h//2)**2
            if dist_to_center < best_dist:
                best_dist = dist_to_center
                best_center = (cx, cy)
                best_box = [cx - bw//2, cy - bh//2, cx + bw//2, cy + bh//2]
                best_cls = cls_id

        return best_center, best_box, best_cls
          
    
           

       
    def get_orientation(self, box):
        """
        根据检测框宽高比判断物体方向
        返回: 0（竖/正方形）或 90（横）
        """
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        if 0.6 < w / h < 1.6:   # 接近正方形
            return 0
        return 90 if w > h else 0
    
    def get_deviation(self, cx, cy, cls_id=0):
        du = cx - self.mid_x - self.offset_x
        dv = cy - self.mid_y - self.offset_y
        if cls_id == 0 and hasattr(self, 'pen_y_offset'):
            dv += self.pen_y_offset
        return du, dv
