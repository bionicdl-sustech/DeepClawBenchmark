# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import socket
import time
import struct
import math
import numpy as np

class URController:
    def __init__(self,robot_ip = "192.168.31.10",port = 30003):
        self.__robot_ip = robot_ip
        self.__port = port
        self.PICK_Z = 0.16
        self.PLACE_Z = 0.172
        self.calibration_tool = ''
        self.goHome()
        time.sleep(5)

    def get_pos(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((self.__robot_ip, self.__port))
            s.send("get_state()"+"\n")
            time.sleep(0.1)
            packet_1 = s.recv(444)

            packet_12 = s.recv(8)
            packet_12 = packet_12.encode("hex") #convert the data from \x hex notation to plain hex
            x = str(packet_12)
            x = struct.unpack('!d', packet_12.decode('hex'))[0]
            # print("X = ", x * 1000)

            packet_13 = s.recv(8)
            packet_13 = packet_13.encode("hex") #convert the data from \x hex notation to plain hex
            y = str(packet_13)
            y = struct.unpack('!d', packet_13.decode('hex'))[0]
            # print("Y = ", y * 1000)

            packet_14 = s.recv(8)
            packet_14 = packet_14.encode("hex") #convert the data from \x hex notation to plain hex
            z = str(packet_14)
            z = struct.unpack('!d', packet_14.decode('hex'))[0]
            # print("Z = ", z * 1000)

            packet_15 = s.recv(8)
            packet_15 = packet_15.encode("hex") #convert the data from \x hex notation to plain hex
            Rx = str(packet_15)
            Rx = struct.unpack('!d', packet_15.decode('hex'))[0]
            # print "Rx = ", Rx

            packet_16 = s.recv(8)
            packet_16 = packet_16.encode("hex") #convert the data from \x hex notation to plain hex
            Ry = str(packet_16)
            Ry = struct.unpack('!d', packet_16.decode('hex'))[0]
            # print "Ry = ", Ry

            packet_17 = s.recv(8)
            packet_17 = packet_17.encode("hex") #convert the data from \x hex notation to plain hex
            Rz = str(packet_17)
            Rz = struct.unpack('!d', packet_17.decode('hex'))[0]
            # print "Rz = ", Rz
            beta = (1-2*3.14/math.sqrt(Rx*Rx+Ry*Ry+Rz*Rz))
            Rx = Rx * beta
            Ry = Ry * beta
            Rz = Rz * beta
            pose = np.zeros(6)
            pose[0] = x*1000.0
            pose[1] = y*1000.0
            pose[2] = z*1000.0
            pose[3] = Rx
            pose[4] = Ry
            pose[5] = Rz
            return pose
            # print("Rx = ", Rx)
            # print("Ry = ", Ry)
            # print("Rz = ", Rz)
            s.close()
        except socket.error as socketerror:
            print("Error: ", socketerror)

    def goHome(self):
        print('homing...')
        self.movej(-0.021, 0.49875, 0.28, math.pi, 0, 0)
    
    def movej(self,x, y, z, Rx, Ry, Rz, a=0.5, v=0.6,useJoint = False):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self.__robot_ip, self.__port))
        # time.sleep(0.05)
        if(useJoint==False):
            # s.send ("movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x/1000.0,y/1000.0,z/1000.0,Rx,Ry,Rz,a,v))
            s.send ("movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x,y,z,Rx,Ry,Rz,a,v))
        else:
            #radian of each joint
            s.send ("movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x*3.14159/180.0,y*3.14159/180.0,z*3.14159/180.0,Rx*3.14159/180.0,Ry*3.14159/180.0,Rz*3.14159/180.0,a,v))
        time.sleep(0.05)
        s.close()

    def verifyPostion(self,targetPosition):
        delay_time = True
        cnt = 0
        timeGap = 0.5
        while(delay_time and cnt < 100):
            currentPose = self.get_pos()
            dpose = np.zeros(6)
            inv_dpose = np.zeros(6)
            dpose[0] = abs(currentPose[0]-targetPosition[0])
            dpose[1] = abs(currentPose[1]-targetPosition[1])
            dpose[2] = abs(currentPose[2]-targetPosition[2])
            dpose[3] = abs(currentPose[3]-targetPosition[3])
            dpose[4] = abs(currentPose[4]-targetPosition[4])
            dpose[5] = abs(currentPose[5]-targetPosition[5])

            inv_dpose[0] = abs(currentPose[0]-targetPosition[0])
            inv_dpose[1] = abs(currentPose[1]-targetPosition[1])
            inv_dpose[2] = abs(currentPose[2]-targetPosition[2])
            inv_dpose[3] = abs(-currentPose[3]-targetPosition[3])
            inv_dpose[4] = abs(-currentPose[4]-targetPosition[4])
            inv_dpose[5] = abs(-currentPose[5]-targetPosition[5])

            if (max(dpose) < 0.2 or max(inv_dpose) < 0.2):
                delay_time = False
                return True
            else:
                time.sleep(timeGap)
                cnt = cnt + 1
            if(cnt*timeGap >= 20):
                print("Time Out!")
                return False


if __name__ == '__main__':
    ip = "192.168.1.10"
    port = 30003
    robotController = URController(ip,port)
    home_tcp_position = [91.02, -87.85, -138.34, -44.5, 90, -0.0,True]
    robotController.movej(home_tcp_position[0], home_tcp_position[1], home_tcp_position[2], home_tcp_position[3], home_tcp_position[4], home_tcp_position[5], 0.4, 0.6,True)
    time.sleep(0.5)
    robotController.verifyPostion(home_tcp_position)
