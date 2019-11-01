# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import numpy as np
import socket
import struct
import time
import math
import sys
import os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from driver.arms.ArmController import ArmController
from input_output.Configuration import *
from modules.calibration.Calibration3D import *


class UR5Controller(ArmController):
    def __init__(self, configuration):
        super(UR5Controller, self).__init__()
        self._cfg = configuration
        self._robot_ip = self._cfg['SOCKET_CONFIGURATION']['robot_ip']
        self._port = self._cfg['SOCKET_CONFIGURATION']['port_number']
        self._home_pose = self._cfg['HOME_POSE']
        self._home_joints = self._cfg['HOME_JOINTS']
        self._pick_z = self._cfg['PICK_Z']
        self._place_z = self._cfg['PLACE_Z']
        self._calibration_tool = ''
        self._R = np.zeros((3, 3))
        self._t = np.zeros((3, 1))

    def goHome(self):
        # print('homing...')
        joint = [self._home_joints[0][0], self._home_joints[0][1], self._home_joints[0][2],
                 self._home_joints[1][0], self._home_joints[1][1], self._home_joints[1][2]]
        pose = [self._home_pose[0][0], self._home_pose[0][1], self._home_pose[0][2],
                self._home_pose[1][0], self._home_pose[1][1], self._home_pose[1][2]]
        self.move([joint, pose], useJoint=True)
        # self.move([[-61.3, -103.53, -139.63], [-26.82, 90, 27.77]], useJoint=True)

    def execute(self, group, plan):
        pass

    def multiple_points_move(self, command):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        s.send(str.encode(command))
        s.close()

    def move(self, goal_pose, a=0.5, v=0.6,useJoint = False):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        # time.sleep(0.05)

        goal_position = goal_pose[0]
        goal_orientation = goal_pose[1]

        if useJoint==False:
            # s.send ("movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x/1000.0,y/1000.0,z/1000.0,Rx,Ry,Rz,a,v))
            x, y, z = goal_position[0], goal_position[1], goal_position[2]
            Rx, Ry, Rz = self.rpy2rotation(goal_orientation[0], goal_orientation[1], goal_orientation[2])
            s.send ("movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x,y,z,Rx,Ry,Rz,a,v))
            self.verifyPosition([x, y, z, Rx, Ry, Rz])
        else:
            #radian of each joint
            x, y, z = goal_position[0], goal_position[1], goal_position[2]
            Rx, Ry, Rz = goal_position[3], goal_position[4], goal_position[5]
            s.send ("movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x*3.14159/180.0,y*3.14159/180.0,z*3.14159/180.0,Rx*3.14159/180.0,Ry*3.14159/180.0,Rz*3.14159/180.0,a,v))
            hx, hy, hz = goal_orientation[0], goal_orientation[1], goal_orientation[2]
            hRx, hRy, hRz = goal_orientation[3], goal_orientation[4], goal_orientation[5]
            self.verifyPosition([hx, hy, hz, hRx, hRy, hRz])
        time.sleep(0.2)
        s.close()

    def openGripper(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        s.send(str.encode('set_digital_out(4,%s)\n' %True))
        s.close()

    def closeGripper(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        s.send(str.encode('set_digital_out(4,%s)\n' %False))
        s.close()

    def verifyPosition(self, targetPosition):
        delay_time = True
        cnt = 0
        timeGap = 1
        while(delay_time and cnt < 100):
            currentPose = self.get_pos()
            # print(targetPosition)
            # print(currentPose)
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

            if (max(dpose) < 0.02 or max(inv_dpose) < 0.02):
                delay_time = False
                return True
            else:
                time.sleep(timeGap)
                cnt = cnt + 1
            if(cnt*timeGap >= 20):
                print("Time Out!")
                return False

    def calibrating3d(self, camera):
        from modules.calibration.Calibration3D import image_callback
        calibration_cfg = readConfiguration(_root_path + "/config/modules/calibration3d.yaml")
        initial_pose = calibration_cfg['initial_pose']
        x_step = calibration_cfg['x_step_length']
        y_step = calibration_cfg['y_step_length']
        z_step = calibration_cfg['z_step_length']

        self.move(initial_pose)
        x = initial_pose[0][0]
        y = initial_pose[0][1]
        z = initial_pose[0][2]

        observed_pts = []
        measured_pts = []
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    self.move([[x + x_step * i, y + y_step * j, z + z_step * k], initial_pose[1]])
                    color_image, info = camera.getImage()
                    depth_image = info[0]
                    observed_pt = image_callback(color_image, depth_image, camera.get_depth_scale())
                    measured_pt = [x + x_step * i, y + y_step * j, z + z_step * k + 0.17]
                    if len(observed_pt) != 0:
                        observed_pts.append(observed_pt)
                        measured_pts.append(measured_pt)
        np.savez(os.path.dirname(_root_path)+self._cfg["CALIBRATION_DIR"], observed_pts, measured_pts)

    def matrix_load(self):
        d = np.load(os.path.dirname(_root_path)+self._cfg["CALIBRATION_DIR"])
        observed_pts = d['arr_0']
        measured_pts = d['arr_1']
        self._R, self._t = self.get_rigid_transform(observed_pts, measured_pts)

    def uvd2xyz(self, u, v, depth_image, depth_scale, intrinsics):
        camera_z = np.mean(np.mean(depth_image[v - 5:v + 5, u - 5:u + 5])) * depth_scale
        camera_x = np.multiply(u - intrinsics.ppx, camera_z / intrinsics.fx)
        camera_y = np.multiply(v - intrinsics.ppy, camera_z / intrinsics.fy)

        view = depth_image[v - 30:v + 30, u - 30:u + 30]
        view[view == 0] = 10000
        avoid_z = np.min(view)

        avoid_v = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[0][0] + v - 5
        avoid_u = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[1][0] + u - 5
        avoid_x = np.multiply(avoid_u - intrinsics.ppx, avoid_z * depth_scale / intrinsics.fx)
        avoid_y = np.multiply(avoid_v - intrinsics.ppy, avoid_z * depth_scale / intrinsics.fy)

        xyz = self._R.dot(np.array([camera_x, camera_y, camera_z]).T) + self._t.T
        avoid_xyz = self._R.dot(np.array([avoid_x, avoid_y, avoid_z * depth_scale]).T) + self._t.T
        return list(xyz.T), avoid_xyz[2]



    def rpy2rotation(self, roll, pitch, yaw):
        yawMatrix = np.matrix([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        pitchMatrix = np.matrix([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = np.matrix([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        R = yawMatrix * pitchMatrix * rollMatrix
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
        rotation = np.zeros(3)
        rotation[0] = rx
        rotation[1] = ry
        rotation[2] = rz
        return rotation

    def get_pos(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((self._robot_ip, self._port))
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
            # pose[0] = x*1000.0
            # pose[1] = y*1000.0
            # pose[2] = z*1000.0
            pose[0] = x
            pose[1] = y
            pose[2] = z
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
    #
    # def get_pos(self):
    #     pose = self.urmonitor.tcf_pose()
    #     return pose