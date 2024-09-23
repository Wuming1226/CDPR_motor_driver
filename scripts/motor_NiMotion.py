#! /usr/bin/env python

import rospy
import os
import can
import time
from calculation_nimotion import relist, decode
import numpy as np

class Motor:
    def __init__(self, canChannel, canID):
        self.canChannel = canChannel
        self.canBus = can.interface.Bus(channel = self.canChannel, bustype = 'socketcan', bitrate=500000)
        self.canID = canID
        self.reqAddress = 0x600 + canID
        self.ansAddress = 0x580 + canID
        
    def __del__(self):
        self.canBus.shutdown()

    def setPosMode(self, position, velocity, relative):

        b,c,d,e = relist(position*10000)
        j,k,m,n = relist(velocity*10000/60)

        # set to CiA402 mode
        msg = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x02, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg)
        time.sleep(0.002)
        # set to contour position mode
        Mode_cho = can.Message(arbitration_id=self.reqAddress, data=[0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(Mode_cho)
        time.sleep(0.002)
        # target position
        Tar_Pos = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x7A, 0x60, 0x00, b, c, d, e],is_extended_id=False)
        self.canBus.send(Tar_Pos)
        time.sleep(0.002)
        # constant profile velocity
        Run_Vol = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x81, 0x60, 0x00, j, k, m, n],is_extended_id=False)
        self.canBus.send(Run_Vol)
        time.sleep(0.002)
        # profile acceleration
        acceleration = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x83, 0x60, 0x00, 0x40, 0x9C, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(acceleration)
        time.sleep(0.002)
        # profile deceleration
        deceleration = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x84, 0x60, 0x00, 0x40, 0x9C, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(deceleration)
        time.sleep(0.002)
        # shutdown
        msg2 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg2)
        time.sleep(0.002)
        # switch on
        msg3 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg3)
        time.sleep(0.002)
        # enable operation
        if relative > 0:
            # relative position command (bit6)
            msg4 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x6F, 0x10, 0x00, 0x00],is_extended_id=False)
            self.canBus.send(msg4)
            time.sleep(0.002)
            # start mission immediately (bit5,4)
            msg5 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x7F, 0x10, 0x00, 0x00],is_extended_id=False)
            self.canBus.send(msg5)
            time.sleep(0.002)
            # info
            print('MotorID: {}    Mode: Position Mode (Reletive)    Position: position'.format(self.canID, position))

        else:
            # absolute position command (bit6)
            msg4 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x2F, 0x10, 0x00, 0x00],is_extended_id=False)
            self.canBus.send(msg4)
            time.sleep(0.002)
            # start mission immediately (bit5,4)
            msg5 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x3F, 0x10, 0x00, 0x00],is_extended_id=False)
            self.canBus.send(msg5)
            time.sleep(0.002)
            # info
            print('MotorID: {}    Mode: Position Mode (Absolute)    Position: position'.format(self.canID, position))
            
        


    '''
    velocity: velocity of motor shaft (-3000 ~ 3000 rpm)
    '''
    def setProfVeloMode(self, velocity):
        
        b,c,d,e = relist(velocity*10000/60)

        # set to CiA402 mode
        msg = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x02, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg)
        time.sleep(0.002)
        # set to velocity mode
        Mode_Cho = can.Message(arbitration_id=self.reqAddress, data=[0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(Mode_Cho)
        time.sleep(0.002)
        # max velocity
        Velo_Max= can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x7F, 0x60, 0x00, 0x20, 0xA1, 0x07, 0x00],is_extended_id=False)
        self.canBus.send(Velo_Max)
        time.sleep(0.002)
        # target velocity
        Tar_Velo = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0xFF, 0x60, 0x00, b, c, d, e],is_extended_id=False)
        self.canBus.send(Tar_Velo)
        time.sleep(0.002)
        # acceleration
        acceleration = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x83, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0xFF],is_extended_id=False)
        self.canBus.send(acceleration) 
        time.sleep(0.002)
        # deceleration
        deceleration = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x84, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0xFF],is_extended_id=False)
        self.canBus.send(deceleration)
        time.sleep(0.002)
        # max acceleration
        max_acceleration = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0xC5, 0x60, 0x00, 0xFF, 0xFF, 0xFF,0xFF],is_extended_id=False)
        self.canBus.send(max_acceleration)
        time.sleep(0.002) 
        # max deceleration
        max_deceleration = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0xC6, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0xFF],is_extended_id=False)
        self.canBus.send(max_deceleration)
        time.sleep(0.002)
        # set to trapezoid slope
        trapezoid = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x86, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(trapezoid)
        time.sleep(0.002)
        #shutdown
        msg2 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg2)
        time.sleep(0.002)
        #switch on
        msg3 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg3)
        time.sleep(0.002)
        # enable operation
        msg4 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg4)
        time.sleep(0.002)
        
        # info
        #print('MotoID: {}    Mode: Profile Velocity Mode    Velocity: {}'.format(self.canID, velocity))
     

    def setTorMode(self, torque, velocity):
        
        b,c,d,e = relist(torque*10)
        #print(b,c,d,e)
        
        j,k,m,n = relist(velocity)
        #print(j,k,m,n)

        msg = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x02, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg)

        Mode_Cho = can.Message(arbitration_id=self.reqAddress, data=[0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(Mode_Cho)
        
        Tar_Tor = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x71, 0x60, 0x00,b, c, d, e],is_extended_id=False)
        self.canBus.send(Tar_Tor)

        Pvelo_Max = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x07, 0x20, 0x10, j, k, m, n],is_extended_id=False)
        self.canBus.send(Pvelo_Max)

        Nvelo_Max = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x07, 0x20, 0x11, j, k, m, n],is_extended_id=False)
        self.canBus.send(Nvelo_Max) 

        Type_Tor = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x88, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(Type_Tor)

        Ramp_Tor = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0x87, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0x0F],is_extended_id=False)
        self.canBus.send(Ramp_Tor)
        
        msg2 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False)
        self.canBus.send(msg2)

        msg3 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg3)

        msg4 = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(msg4)

     	  	
    def stop(self):
        stop = can.Message(arbitration_id=self.reqAddress, data=[0x2B, 0x40, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00],is_extended_id=False)
        self.canBus.send(stop)


    def setVelo(self, velocity):
    
        self.canBus = can.interface.Bus(channel = self.canChannel, bustype = 'socketcan')
    
        b, c, d, e = relist(velocity*10000/60)
        Tar_Velo = can.Message(arbitration_id=self.reqAddress, data=[0x23, 0xFF, 0x60, 0x00, b, c, d, e], is_extended_id=False)
        self.canBus.send(Tar_Velo)
        time.sleep(0.02)

        # info
        #print('MotoID: {}    set Velocity: {}'.format(self.canID, velocity))


    def getVelo(self):
    
        self.canBus = can.interface.Bus(channel = self.canChannel, bustype = 'socketcan')
        
        # request
        req = can.Message(arbitration_id=self.reqAddress,  data=[0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

        while True:
            self.canBus.send(req)
            time.sleep(0.002)
            ans = self.canBus.recv()

            if(ans.arbitration_id == self.ansAddress):
                data = decode(ans.data)
                break

        return data


    def getPos(self):
    
        self.canBus = can.interface.Bus(channel = self.canChannel, bustype = 'socketcan')

        req = can.Message(arbitration_id=self.reqAddress, data=[0x40, 0X64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
        
        while True:
            self.canBus.send(req)
            time.sleep(0.002)
            ans = self.canBus.recv()

            if (ans.arbitration_id == self.ansAddress):
                data = decode(ans.data)
                break

        return data
        




if __name__=="__main__":

    try:
        rospy.init_node('motor', anonymous=True)
        
        os.system('sudo ip link set can0 type can bitrate 500000')
        os.system('sudo ifconfig can0 up')
        rate = rospy.Rate(10)

        motor1 = Motor('can0', 3)
#        motor1.setPosMode(-3845038, 300, False)
        
        #while True:
        #    print(motor1.getPos())
        #    rate.sleep()
        
        
        time.sleep(1)
        motor1.setProfVeloMode(-200)
        time.sleep(5)
        #motor1.setVelo(100)
        #time.sleep(1.5)
        motor1.setProfVeloMode(0)

#        cnt = 0
#        while cnt < 20:
#            print("Velocity: {}".format(motor1.getVelo()))
#            rate.sleep()
#            cnt += 1
#        motor1.stop()


        
    except rospy.ROSInterruptException:
        motor1.stop()

    
   
    

