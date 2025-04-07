#! /usr/bin/python3

import os
import can
import struct
import time
import numpy as np


class Motor:
    def __init__(self, master_id, can_id):
        self._master_id = master_id
        self._can_id = can_id
        self._can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        self._run_mode = 0
        self._overflow_cnt = 0
        
        self.set_zero_pos()

    def __del__(self):
        self._can_bus.shutdown()

    """
    电机使能运行（通信类型3）
    """
    def enable(self):
        # 扩展帧id
        msg_id = (3 << 24) + (self._master_id << 8) + self._can_id
        # 数据
        msg_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

    """
    电机停止运行（通信类型4）
    注意：此电机无刹车，停止运行后无转矩输出，若有负载则无法保持位置！
    """
    def stop(self):
        # 扩展帧id
        msg_id = (4 << 24) + (self._master_id << 8) + self._can_id
        # 数据
        msg_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

    """
    设置运行模式（通信类型18）
    """
    def set_run_mode(self, run_mode):
        # 扩展帧id
        msg_id = (18 << 24) + (self._master_id << 8) + self._can_id
        # 数据
        msg_data = [0x05, 0x70, 0x00, 0x00, run_mode, 0x00, 0x00, 0x00]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

        # 反馈检测
        if self.get_response_msg():
            self._run_mode = run_mode
            return True
        else:
            return False

    """
    设置运控模式参数（通信类型1)
      output_torqur = torque + Kp * position_err + Kd * velocity_err
    """
    def motion_control(self, torque, position, velocity, Kp, Kd):
        # 判断是否在运控模式
        if self._run_mode != 0:
            print('Not in motion control mode!')
            return False

        # 数据限幅及标准化
        torque = 12 if torque > 12 else (-12 if torque < -12 else torque)
        torque = int((torque + 12) / 24 * 65535)
        position = 4*np.pi if position > 4*np.pi else (-4*np.pi if position < -4*np.pi else position)
        position = int((position + 4*np.pi) / (8*np.pi) * 65535)
        velocity = 30 if velocity > 30 else (-30 if velocity < -30 else velocity)
        velocity = int((velocity + 30) / 60 * 65535)
        Kp = 500 if Kp > 500 else (0 if Kp < 0 else Kp)
        Kp = int(Kp / 500 * 65535)
        Kd = 5 if Kd > 5 else (0 if Kd < 0 else Kd)
        Kd = int(Kd / 5 * 65535)

        # 扩展帧id
        msg_id = (1 << 24) + (torque << 8) + self._can_id
        # 数据（大端uint16）
        msg_data = [position >> 8, position - ((position >> 8) << 8),
                velocity >> 8, velocity - ((velocity >> 8) << 8),
                Kp >> 8, Kp - ((Kp >> 8) << 8),
                Kd >> 8, Kd - ((Kd >> 8) << 8)]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

        # 反馈检测
        return self.get_response_msg()

    """
    设置目标速度（速度模式）（通信类型18）
    """
    def set_velocity(self, target_velocity):
        # 判断是否在速度模式
        if self._run_mode != 2:
            print('Not in velocity mode!')
            return False

        # 速度限幅（-30~30 rad/s）
        target_velocity = 30 if target_velocity > 30 else (-30 if target_velocity < -30 else target_velocity)
        # 扩展帧id
        msg_id = (18 << 24) + (self._master_id << 8) + self._can_id
        # 数据（小端float）
        data_byte = struct.pack('<f', target_velocity)                      # 打包成小端字节流
        data_hex = hex(int.from_bytes(data_byte, 'big'))[2:].zfill(8)          # 将字节流转为小端十六进制（32bit，8位十六进制）
        msg_data = [0x0A, 0x70, 0x00, 0x00,
                int(data_hex[0:2], 16), int(data_hex[2:4], 16), int(data_hex[4:6], 16), int(data_hex[6:8], 16)]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

        # 反馈检测
        return self.get_response_msg()

    """
    接收反馈消息（通信类型2）
    """
    def get_response_msg(self):
        # 接收消息，验证消息类型
        time_out = 1
        rcv_start_time = time.time()
        msg = self._can_bus.recv(0.1)
        while time.time() - rcv_start_time < time_out:
            if msg is not None:
                if (msg.arbitration_id >> 24) == 2:
                    break
            msg = self._can_bus.recv(0.1)
        if msg is None:
            print('No response!')
            return False       

        # 帧id信息
        motor_id = (msg.arbitration_id >> 8) - ((msg.arbitration_id >> 16) << 8)
        error_flag = (msg.arbitration_id >> 16) - ((msg.arbitration_id >> 22) << 6)
        mode_id = (msg.arbitration_id >> 22) - ((msg.arbitration_id >> 24) << 2)
        mode = 'Reset' if mode_id == 0 else ('Cali' if mode_id == 1 else 'Motor')

        # 数据信息
        data_hex = hex(int.from_bytes(msg.data, 'big'))[2:].zfill(16)
        position = (int(data_hex[0:2], 16) << 8) + int(data_hex[2:4], 16)
        velocity = (int(data_hex[4:6], 16) << 8) + int(data_hex[6:8], 16)
        torque = (int(data_hex[8:10], 16) << 8) + int(data_hex[10:12], 16)
        temperature = (int(data_hex[12:14], 16) << 8) + int(data_hex[14:16], 16)

        position = position / 65535 * 8*np.pi - 4*np.pi
        velocity = velocity / 65535 * 60 - 30
        torque = torque / 65535 * 24 - 12
        temperature = temperature / 10

        status = {'position': position, 'velocity': velocity, 'torque': torque, 'temperature': temperature}

        # 打印状态
        if error_flag == 0:
            # print('Motor_id: {}  No error.  Mode: {}'.format(motor_id, mode))
            # print('Pos: {}  Vel: {}  Toq: {}'.format(position, velocity, torque))
            return status
        else:
            print('Motor_id: {}  Error!'.format(motor_id))
            return False
    
    """
    读取电机参数（通信类型17）
    """
    def get_param(self, param_index):
        # 扩展帧id
        msg_id = (17 << 24) + (self._master_id << 8) + self._can_id
        # 数据
        msg_data = [param_index, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

        # 接收消息，验证消息类型
        time_out = 0.5
        rcv_start_time = time.time()
        msg = self._can_bus.recv(0.1)
        while time.time() - rcv_start_time < time_out:
            if msg is not None:
                if (msg.arbitration_id >> 24) == 17:
                    break
            msg = self._can_bus.recv(0.1)
        if msg is None:
            print('No response!')
            return False

        # 数据信息
        if param_index == 0x05:
            data_int = int.from_bytes(msg.data, 'little')       # 将小端字节流转为整型
            data_int = data_int >> 32                           # 取出数据（msg.data的Byte4）
            return data_int - ((data_int >> 8) << 8)
        else:
            data_int = int.from_bytes(msg.data, 'little')       # 将小端字节流转为整型
            data_int = data_int >> 32                           # 取出数据（msg.data的Byte4~7）
            data_byte = data_int.to_bytes(4, 'big')             # 重新打包成大端字节流
            return struct.unpack('>f', data_byte)[0]            # 将大端字节流转为浮点型

    """
    设置电机机械零位（通信类型6）
    """
    def set_zero_pos(self):
        # 扩展帧id
        msg_id = (6 << 24) + (self._master_id << 8) + self._can_id
        # 数据
        msg_data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)
        
        # 反馈检测
        return self.get_response_msg()

    """
    设置最大电流（通信类型18）
    PMSM 转矩与iq成正比，经测试电流与转矩的数值之比约为 1.67:1
    """
    def set_max_cur(self, max_cur):
        # 扩展帧id
        msg_id = (18 << 24) + (self._master_id << 8) + self._can_id
        # 数据
        data_byte = struct.pack('<f', max_cur)  # 打包成小端字节流
        data_hex = hex(int.from_bytes(data_byte, 'big'))[2:].zfill(8)  # 将字节流转为小端十六进制（32bit，8位十六进制）
        msg_data = [0x18, 0x70, 0x00, 0x00,
                    int(data_hex[0:2], 16), int(data_hex[2:4], 16), int(data_hex[4:6], 16), int(data_hex[6:8], 16)]

        # 生成数据帧并发送
        msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=True)
        self._can_bus.send(msg)

        # 反馈检测
        if self.get_response_msg():
            self._run_mode = run_mode
            return True
        else:
            return False
        

if __name__ == '__main__':
    os.system('sudo ip link set can0 type can bitrate 1000000')     # 电机CAN通信波特率100M
    os.system('sudo ifconfig can0 up')

    # test
    motor = Motor(111, 2)
    motor.set_run_mode(2)
    motor.enable()
    start = time.time()
    while time.time() - start < 1:
        status = motor.set_velocity(-0.2)
        position = status['position']
        
        # 下面这一堆封装到cdpr
        if position >= 4*np.pi:
            motor.set_zero_pos()
            motor._overflow_cnt += 1
            position = 0
        elif position <= -4*np.pi:
            motor.set_zero_pos()
            motor._overflow_cnt -= 1
            position = 0
        elif np.abs(position) < 0.001 and motor._overflow_cnt != 0: 
            motor._overflow_cnt -= np.sign(motor._overflow_cnt)
            position = 4*np.pi * np.sign(motor._overflow_cnt)

        position = position + 4*np.pi * motor._overflow_cnt
        print(position)

        time.sleep(0.01)
        
    start = time.time()
    while time.time() - start < 0:
        status = motor.set_velocity(3)
        position = status['position']
        
        # 下面这一堆封装到cdpr
        if position >= 4*np.pi:
            motor.set_zero_pos()
            motor._overflow_cnt += 1
            position = 0
        elif position <= -4*np.pi:
            motor.set_zero_pos()
            motor._overflow_cnt -= 1
            position = 0
        elif np.abs(position) < 0.001 and motor._overflow_cnt != 0: 
            motor._overflow_cnt -= np.sign(motor._overflow_cnt)
            position = 4*np.pi * np.sign(motor._overflow_cnt)

        position = position + 4*np.pi * motor._overflow_cnt
        print(position)

        time.sleep(0.01)
    motor.stop()
    
    # motor.set_run_mode(0)
    # motor.enable()
    # motor.motion_control(0.05, 0, 0, 0, 0)
    # time.sleep(2)
    # motor.stop()
    
    #start = time.time()
    #while time.time() - start < 10:
    #    motor.get_response_msg()
    #    motor.get_encoder()
    #    time.sleep(0.5)

    #while time.time() - start < 10:
    #    motor.get_response_msg()
    #    motor.get_encoder()
    #    time.sleep(0.5)
