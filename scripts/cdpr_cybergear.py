#! /usr/bin/env python3

import os
import rospy
import time
import numpy as np

from motor_cybergear import Motor

from std_msgs.msg import Float32MultiArray


class CDPR:
    def __init__(self, master_id, motor1_id, motor2_id, motor3_id, motor4_id):

        # can settings0
        os.system('sudo ip link set can0 type can bitrate 1000000')
        os.system('sudo ifconfig can0 up')
        self.canChannel = 'can0'

        # initialize motors
        self.motor1 = Motor(master_id, motor1_id)
        if self.motor1.set_run_mode(1):     # 设置为位置模式，进入位置模式时，电机自动将当前位置设为目标位置
            self.motor1.enable()
            self.motor1.set_max_cur(10)     # 设置最大电流限制输出转矩
        else:
            print('motor1 setting failed')
            exit()

        self.motor2 = Motor(master_id, motor2_id)
        if self.motor2.set_run_mode(1):     # 设置为位置模式
            self.motor2.enable()
            self.motor2.set_max_cur(10)
        else:
            print('motor2 setting failed')
            exit()

        self.motor3 = Motor(master_id, motor3_id)
        if self.motor3.set_run_mode(1):     # 设置为位置模式
            self.motor3.enable()
            self.motor3.set_max_cur(10)
        else:
            print('motor3 setting failed')
            exit()

        self.motor4 = Motor(master_id, motor4_id)
        if self.motor4.set_run_mode(1):     # 设置为位置模式
            self.motor4.enable()
            self.motor4.set_max_cur(10)
        else:
            print('motor4 setting failed')
            exit()

        self.motor_list = [self.motor1, self.motor2, self.motor3, self.motor4]
        self.motor_overflow_cnt_list = np.array([0, 0, 0, 0])

        # ros settings
        rospy.init_node('cdpr_actuate', anonymous=True)
        rospy.Subscriber('motor_velo', Float32MultiArray, callback=self.velo_callback)
        self._motorPosPub = rospy.Publisher('motor_pos', Float32MultiArray, queue_size=10)

        # safety
        self.max_interval = 0.5
        self.last_velo_cb_time = time.time()
        self.max_velo = 2
        self.exceed_cnt = 0
        self.exceed_tol = 10

    def velo_callback(self, msg):
        self.last_velo_cb_time = time.time()
        print(msg.data[0], msg.data[1], msg.data[2], msg.data[3])

        # 最大速度指令计数
        if np.abs(msg.data[0]) >= self.max_velo or np.abs(msg.data[1]) >= self.max_velo or np.abs(
                msg.data[2]) >= self.max_velo or np.abs(msg.data[3]) >= self.max_velo:
            self.exceed_cnt += 1
        else:
            self.exceed_cnt -= 1
            self.exceed_cnt = 0 if self.exceed_cnt < 0 else self.exceed_cnt
            
        if self.exceed_cnt > self.exceed_tol:
            status1 = self.motor1.set_run_mode(1)
            status2 = self.motor2.set_run_mode(1)
            status3 = self.motor3.set_run_mode(1)
            status4 = self.motor4.set_run_mode(1)
        else:
            ### 速度控制似乎有问题，因为无法保证平衡位置，可以考虑更换为全程位置控制
            self.motor1.set_run_mode(2)
            self.motor1.enable()
            status1 = self.motor1.set_velocity(msg.data[0])
            self.motor2.set_run_mode(2)
            self.motor2.enable()
            status2 = self.motor2.set_velocity(msg.data[1])
            self.motor3.set_run_mode(2)
            self.motor3.enable()
            status3 = self.motor3.set_velocity(msg.data[2])
            self.motor4.set_run_mode(2)
            self.motor4.enable()
            status4 = self.motor4.set_velocity(msg.data[3])

        # positions = np.array([status1['position'], status2['position'], status3["position"], status4['position']])
        # self.calculate_motor_position(positions)

    # def calculate_motor_position(self, positions):
    #     for index in range(4):
    #         if positions[index] >= 4*np.pi:     # 若达到正位置最大值
    #             motor = self.motor_list[index]
    #             motor.set_zero_pos()            # 重新设置零位
    #             self.motor_overflow_cnt_list[index] += 1    # 正向溢出计数
    #             positions[index] = 0
    #         elif positions[index] <= -4*np.pi:  # 若达到负位置最大值
    #             motor = self.motor_list[index]
    #             motor.set_zero_pos()            # 重新设置零位
    #             self.motor_overflow_cnt_list[index] -= 1    # 负向溢出计数
    #             positions[index] = 0
    #
    #     positions = positions + 4*np.pi * self.motor_overflow_cnt_list
    #     print('positions: {}'.format(positions))
    #     self.pub_motor_pos(positions)
    #     return positions

    def get_and_pub_motor_pos(self):
        positions = [self.motor1.get_position(), self.motor2.get_position(), self.motor3.get_position(), self.motor4.get_position()]
        motor_pos = Float32MultiArray(data=positions)
        self._motorPosPub.publish(motor_pos)


if __name__ == "__main__":

    cdpr = CDPR(master_id=111, motor1_id=1, motor2_id=2, motor3_id=3, motor4_id=4)

    rate = rospy.Rate(20)
    exceed_cnt = 0
    time.sleep(1)
    while not rospy.is_shutdown():

        if time.time() - cdpr.last_velo_cb_time > cdpr.max_interval:  # 规定时间间隔内没接收到速度指令则停机
            cdpr.motor1.set_run_mode(1)     # 切换为位置模式，锁定电机位置
            cdpr.motor2.set_run_mode(1)
            cdpr.motor3.set_run_mode(1)
            cdpr.motor4.set_run_mode(1)
            print('No signal')

        if cdpr.exceed_cnt >= cdpr.exceed_tol:  # 连续接收最大速度指令，则判断为发生错误，停机
            cdpr.motor1.set_run_mode(1)  # 切换为位置模式，锁定电机位置
            cdpr.motor2.set_run_mode(1)
            cdpr.motor3.set_run_mode(1)
            cdpr.motor4.set_run_mode(1)
            print('Over speed')

        cdpr.get_and_pub_motor_pos()
        rate.sleep()

    cdpr.motor1.stop()      # 电机停机后失去输出转矩，由于没有物理刹车，末端执行器将下坠
    cdpr.motor2.stop()
    cdpr.motor3.stop()
    cdpr.motor4.stop()
