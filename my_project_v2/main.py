#!/usr/bin/env python3
"""
新框架主入口 —— 编排脚本
只描述流程：找盒子 → 找物体 → 伺服 → 抓取 → 放置 → 归位 → 循环
"""
import threading
import sys
import time
import yaml

from hardware.ik_engine import Kinematics
from hardware.camera import Camera
from hardware.primitives import MotionPrimitives
from perception.detector import Detector
from perception.box_locator import find_marker
from core.action_executor import ActionExecutor
from core.task_planner import TaskPlanner


def load_config(path="config.yaml"):
    """加载配置文件"""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

class TaskTimeout(Exception):
    pass

def run_with_timeout(func, timeout, *args, **kwargs):
    """在 timeout 秒内执行 func，超时抛出 TaskTimeout"""
    result = [None]
    exc = [None]
    def target():
        try:
            result[0] = func(*args, **kwargs)
        except Exception as e:
            exc[0] = e
    t = threading.Thread(target=target)
    t.daemon = True
    t.start()
    t.join(timeout)
    if t.is_alive():
        raise TaskTimeout(f"任务超时({timeout}秒)")
    if exc[0]:
        raise exc[0]
    return result[0]




def main():
    # 1. 加载配置
    config = load_config()

    # 2. 平台自适应
    if sys.platform.startswith('win'):
        port = sys.argv[1] if len(sys.argv) > 1 else config.get('port_win', 'COM6')
        cam_id = int(sys.argv[2]) if len(sys.argv) > 2 else config.get('camera_win', 1)
    else:
        port = sys.argv[1] if len(sys.argv) > 1 else config.get('port_linux', '/dev/ttyS0')
        cam_id = int(sys.argv[2]) if len(sys.argv) > 2 else config.get('camera_linux', 20)

    # 3. 初始化硬件
    print(f"连接机械臂: {port}")
    arm = Kinematics(port)
    camera = Camera(camera_id=cam_id)
    motion = MotionPrimitives(arm, home_xyz=tuple(config['home']))

    # 4. 初始化感知
    detector = Detector(
        config['model_path'],
        input_size=config['input_size'],
        conf_thresh=config['conf_thresh'],
        offset_x=config.get('offset_x', 45),
        offset_y=config.get('offset_y', 350),
        pen_y_offset=config.get('pen_y_offset', 100)
    )

    # 5. 初始化执行层和决策层
    executor = ActionExecutor(config, motion, camera, detector, find_marker)
    planner = TaskPlanner(config)
    if config.get('enable_stream', False):
        executor.start_stream_server(port=config.get('stream_port', 8080))

    # 6. 归位
    print("开始归位...")
    motion.gripper_open()
    motion.rotate_wrist(0)
    motion.go_home()
    time.sleep(1)
        # 6. 归位
    print("开始归位...")           # ★ 加这行
    motion.gripper_open()
    motion.rotate_wrist(0)
    motion.go_home()
    time.sleep(1)
    print("归位完成，系统就绪！")   # ★ 改这行

    # 7. 主循环
    while True:
        task = planner.next_task()
        try:
            if task == "find_boxes":
                boxes = run_with_timeout(executor.find_boxes, 30)
                planner.on_boxes_found(boxes['blue'], boxes['orange'])

            elif task == "scan":
                result = run_with_timeout(executor.scan, 60)
                if result['found']:
                    planner.on_object_found(
                        (result['x'], result['y']),
                        result['class'],
                        result['orientation']
                    )
                else:
                    print("扫描完成，未找到物体，归位")
                    executor.go_home()
                    print("程序结束")
                    break   # 退出循环，程序结束

            elif task == "servo":
                start_xy = planner.object_xy
                final_xy = run_with_timeout(executor.servo, 20, start_xy)
                if final_xy:
                    planner.on_servo_done(final_xy)

            elif task == "pick":
                info = planner.get_object_info()
                run_with_timeout(executor.pick, 15, info['xy'], info['orientation'], info.get('cls_id', -1))
                planner.on_pick_done()

            elif task == "place":
                class_name = planner.object_class
                box_xy = planner.get_box_xy(class_name)
                if box_xy:
                    run_with_timeout(executor.place, 15, box_xy)
                else:
                    run_with_timeout(executor.place, 15, planner.box_positions.get('blue', (0, 200)))
                planner.on_place_done()

            elif task == "home":
                executor.go_home()
                time.sleep(1)
                print("=" * 50)
                print("一轮完成，开始下一轮扫描...")
                print("=" * 50)
                planner._reset()   # 重置状态，下一轮自动从 scan 开始

            elif task == "done":
                break

        except TaskTimeout as e:
            print(f"任务 {task} 超时: {e}，紧急归位并重置")
            executor.go_home()
            planner._reset()
            continue

    # 8. 安全退出
    executor.go_home()
    arm.close()
    camera.release()
    print("程序结束")


if __name__ == "__main__":
    main()
