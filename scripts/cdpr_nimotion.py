#! /usr/bin/env python3

import os
import rospy
import time
import numpy as np

from motor import Motor

from std_msgs.msg import Int16MultiArray, Int64MultiArray


class CDPR:
    def __init__(self, motor1_id, motor2_id, motor3_id, motor4_id):

        # can settings0
        os.system('sudo ip link set can0 type can bitrate 500000')
        os.system('sudo ifconfig can0 up')
        self.canChannel = 'can0'

        # initialize motors
        self.motor1 = Motor(self.canChannel, motor1_id)
        self.motor1.setProfVeloMode(0)
        self.motor2 = Motor(self.canChannel, motor2_id)
        self.motor2.setProfVeloMode(0)
        self.motor3 = Motor(self.canChannel, motor3_id)
        self.motor3.setProfVeloMode(0)
        self.motor4 = Motor(self.canChannel, motor4_id)
        self.motor4.setProfVeloMode(0)

        # ros settings
        rospy.init_node('cdpr_actuate', anonymous=True)
        rospy.Subscriber('motor_velo', Int16MultiArray, callback=self.velo_callback)
        self._motorPosPub = rospy.Publisher('motor_pos', Int64MultiArray, queue_size=10)

        # safety
        self.max_interval = 0.5
        self.last_velo_cb_time = time.time()
        self.max_velo = 600
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
            self.motor1.setVelo(0)
            self.motor2.setVelo(0)
            self.motor3.setVelo(0)
            self.motor4.setVelo(0)
        else:
            self.motor1.setVelo(msg.data[0])
            self.motor2.setVelo(msg.data[1])
            self.motor3.setVelo(msg.data[2])
            self.motor4.setVelo(msg.data[3])

    def pub_motor_pos(self):
        motor_pos = Int64MultiArray(
            data=np.array([self.motor1.getPos(), self.motor2.getPos(), self.motor3.getPos(), self.motor4.getPos()]))
        self._motorPosPub.publish(motor_pos)


if __name__ == "__main__":

    cdpr = CDPR(motor1_id=1, motor2_id=2, motor3_id=3, motor4_id=4)

    rate = rospy.Rate(20)
    exceed_cnt = 0
    time.sleep(1)
    while not rospy.is_shutdown():
        cdpr.pub_motor_pos()

        if time.time() - cdpr.last_velo_cb_time > cdpr.max_interval:  # 规定时间间隔内没接收到速度指令则停机
            cdpr.motor1.setVelo(0)
            cdpr.motor2.setVelo(0)
            cdpr.motor3.setVelo(0)
            cdpr.motor4.setVelo(0)
            print('No signal')

        if cdpr.exceed_cnt >= cdpr.exceed_tol:  # 连续接收最大速度指令，则判断为发生错误，停机
            print('Over speed')
            exit()

        rate.sleep()