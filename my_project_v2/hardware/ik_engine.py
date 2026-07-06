#!/usr/bin/env python3
"""
逆运动学引擎 —— 原厂算法，100%兼容STM32固件
提供：坐标→舵机PWM指令的转换
"""

import math
import time
import serial


class Kinematics:
    # 四个关节长度（放大10倍，单位0.1mm）
    L0 = 1000
    L1 = 1050
    L2 = 880
    L3 = 1550
    pi = 3.1415926

    def __init__(self, port='/dev/ttyS0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        self.move_time = 1500

    def send_str(self, cmd: str):
        self.ser.write(cmd.encode())

    def kinematics_analysis(self, x: float, y: float, z: float, Alpha: float) -> str:
        """逆运动学解算，返回多舵机PWM指令字符串"""
        x *= 10
        y *= 10
        z *= 10

        l0 = float(self.L0)
        l1 = float(self.L1)
        l2 = float(self.L2)
        l3 = float(self.L3)

        # ★ theta6：修正为乘以180.0（和C固件一致）
        if x == 0:
            theta6 = 0.0
        else:
            theta6 = math.atan(x / y) * 180.0 / self.pi

        y = math.sqrt(x * x + y * y)
        y = y - l3 * math.cos(Alpha * self.pi / 180.0)
        z = z - l0 - l3 * math.sin(Alpha * self.pi / 180.0)

        if z < -l0:
            return None
        if math.sqrt(y * y + z * z) > (l1 + l2):
            return None

        ccc = math.acos(y / math.sqrt(y * y + z * z))
        bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2 * l1 * math.sqrt(y * y + z * z))
        if bbb > 1 or bbb < -1:
            return None

        zf_flag = -1 if z < 0 else 1
        theta5 = ccc * zf_flag + math.acos(bbb)
        theta5 = theta5 * 180.0 / self.pi
        if theta5 > 180.0 or theta5 < 0.0:
            return None

        aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        if aaa > 1 or aaa < -1:
            return None

        theta4 = math.acos(aaa)
        theta4 = 180.0 - theta4 * 180.0 / self.pi
        if theta4 > 135.0 or theta4 < -135.0:
            return None

        theta3 = Alpha - theta5 + theta4
        if theta3 > 90.0 or theta3 < -90.0:
            return None

        servo_angle0 = theta6
        servo_angle1 = theta5 - 90
        servo_angle2 = theta4
        servo_angle3 = theta3

        servo_pwm0 = int(1500 - 2000.0 * servo_angle0 / 270.0)
        servo_pwm1 = int(1500 + 2000.0 * servo_angle1 / 270.0)
        servo_pwm2 = int(1500 + 2000.0 * servo_angle2 / 270.0)
        servo_pwm3 = int(1500 - 2000.0 * servo_angle3 / 270.0)

        arm_str = ("{{#000P{0:0>4d}T{4:0>4d}!#001P{1:0>4d}T{4:0>4d}!"
                   "#002P{2:0>4d}T{4:0>4d}!#003P{3:0>4d}T{4:0>4d}!}}".format(
                       servo_pwm0, servo_pwm1, servo_pwm2, servo_pwm3, self.move_time))
        return arm_str

    def kinematics_move(self, x: float, y: float, z: float, time_ms: int = 1000) -> bool:
        """移动机械臂末端到目标坐标，返回True/False"""
        self.move_time = time_ms
        if y <= 0:
            y = 1

        best_alpha = 0
        found = False
        for i in range(0, -136, -1):
            result = self.kinematics_analysis(x, y, z, i)
            if isinstance(result, str):
                if not found or i < best_alpha:
                    best_alpha = i
                    found = True

        if found:
            cmd = self.kinematics_analysis(x, y, z, best_alpha)
            if cmd:
                self.send_str(cmd)
                return True
        return False

    def close(self):
        self.ser.close()