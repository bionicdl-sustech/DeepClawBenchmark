# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import socket
import struct
import math
import time
import sys
import os
from scipy.spatial.transform import Rotation as R


_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)

from driver.arms.ArmController import ArmController
from input_output.Configuration import *
from modules.calibration.Calibration3D import *
from driver.sensors.FT.getFT_Data import detectCollision


class UR5Controller(ArmController):
    def __init__(self, configuration_path):
        super(UR5Controller, self).__init__()
        self._cfg = readConfiguration(configuration_path)
        self._robot_ip = self._cfg['SOCKET_CONFIGURATION']['robot_ip']
        self._port = self._cfg['SOCKET_CONFIGURATION']['port_number']
        self._home_pose = self._cfg['HOME_POSE']
        self._home_joints = self._cfg['HOME_JOINTS']
        self._pick_z = self._cfg['PICK_Z']
        self._place_z = self._cfg['PLACE_Z']
        self._R = None
        self._t = None

        # self.load_calibration_matrix()

    def go_home(self):
        joint = [self._home_joints[0], self._home_joints[1], self._home_joints[2],
                 self._home_joints[3], self._home_joints[4], self._home_joints[5]]
        self.move_j(joint)

        #the unit of joint is degrees
    def move_j(self, joint, velocity=0.5, accelerate=0.6, solution_space="Joint"):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))

        if solution_space=="Joint":
            # move_command = f"movej([{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movej([{},{},{},{},{},{}],a={},v={})\n".format(joint[0]*3.14159/180.0, joint[1]*3.14159/180.0, joint[2]*3.14159/180.0,
                                                                           joint[3]*3.14159/180.0, joint[4]*3.14159/180.0, joint[5]*3.14159/180.0,
                                                                           accelerate, velocity)
        else:
            # move_command = f"movel([{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movel([{},{},{},{},{},{}],a={},v={})\n".format(joint[0]*3.14159/180.0, joint[1]*3.14159/180.0, joint[2]*3.14159/180.0,
                                                                           joint[3]*3.14159/180.0, joint[4]*3.14159/180.0, joint[5]*3.14159/180.0,
                                                                           accelerate, velocity)
        # move_command = bytes(move_command, encoding='utf-8')
        s.send(move_command)
        s.close()
        collosion_bool = self.verify_state("Joint", joint,error = 1,FT = False)
        # collosion_bool is True, means the collision is happended
        return collosion_bool

    def move_p(self, position, velocity=0.5, accelerate=0.6, solution_space="Joint"):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        # Rx, Ry, Rz = self.rpy2rotation(position[3], position[4], position[5])
        r = R.from_euler('xyz', [position[3], position[4], position[5]], degrees=False)
        Rx, Ry, Rz = r.as_rotvec()

        if solution_space == "Joint":
            # move_command = f"movej([p{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movej(p[{},{},{},{},{},{}],a={},v={})\n".format(position[0], position[1], position[2],
                                                                            Rx, Ry, Rz,
                                                                            accelerate, velocity)
        else:
            # move_command = f"movel([p{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movel(p[{},{},{},{},{},{}],a={},v={})\n".format(position[0], position[1], position[2],
                                                                            Rx, Ry, Rz,
                                                                            accelerate, velocity)
        # move_command = bytes(move_command, encoding='utf-8')
        s.send(move_command)
        s.close()
        collosion_bool = self.verify_state("Position", position, error=0.01,FT = False)
        return collosion_bool

    def set_io(self, port, value):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        # command = f"set_digital_out({port}, {value})\n"
        # command = bytes(command, encoding='utf-8')
        command = "set_digital_out({}, {})\n".format(port, value)
        s.send(command)
        s.close()

    def get_state(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        # command = f"get_state()\n"
        # command = bytes(command, encoding='utf-8')
        command = "get_state()\n"
        s.send(command)

        packet_1 = s.recv(252)
        joint_1 = self.encode_information(s) / 3.14 * 180
        joint_2 = self.encode_information(s) / 3.14 * 180
        joint_3 = self.encode_information(s) / 3.14 * 180
        joint_4 = self.encode_information(s) / 3.14 * 180
        joint_5 = self.encode_information(s) / 3.14 * 180
        joint_6 = self.encode_information(s) / 3.14 * 180

        packet_2 = s.recv(144)
        x = self.encode_information(s)
        y = self.encode_information(s)
        z = self.encode_information(s)
        Rx = self.encode_information(s)
        Ry = self.encode_information(s)
        Rz = self.encode_information(s)

        #transfer to euler angle
        r = R.from_rotvec([Rx, Ry, Rz])
        euler_angle = r.as_euler('xyz',degrees=False)

        return {"Position": [x, y, z, euler_angle[0], euler_angle[1], euler_angle[2]], "Joint": [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]}

    def verify_state(self, variable_name, target_value,error=0.002,FT = False):
        delay_time = True
        cnt = 0
        timeGap = 0.25
        while(delay_time and cnt < 100):
            currentStatus = self.get_state()
            currentPose = currentStatus[variable_name]
            if currentPose is None:
                print("Getting current value failed, please check variable name.")
                return False
            dpose = np.zeros(6)
            dpose[0] = abs(currentPose[0]-target_value[0])
            dpose[1] = abs(currentPose[1]-target_value[1])
            dpose[2] = abs(currentPose[2]-target_value[2])
            dpose[3] = abs(currentPose[3]-target_value[3])
            if dpose[3]>6:
                dpose[3] = dpose[3] - 2* 3.141592653589793
            elif dpose[3]<-6:
                dpose[3] = dpose[3] + 2* 3.141592653589793
            else:
                pass
            dpose[4] = abs(currentPose[4]-target_value[4])
            if dpose[4]>6:
                dpose[4] = dpose[4] - 2* 3.141592653589793
            elif dpose[4]<-6:
                dpose[4] = dpose[4] + 2* 3.141592653589793
            else:
                pass
            dpose[5] = abs(currentPose[5]-target_value[5])
            if dpose[5]>6:
                dpose[5] = dpose[5] - 2* 3.141592653589793
            elif dpose[5]<-6:
                dpose[5] = dpose[5] + 2* 3.141592653589793
            else:
                pass

            if(FT == True):
                collosion_bool = detectCollision()
                if collosion_bool==True:
                    return False
                else:
                    pass

            if (max(dpose) < error):
                delay_time = False
                return True
            else:
                time.sleep(timeGap)
                cnt = cnt + 1
            if(cnt*timeGap >= 20):
                print("Time Out!")
                return False

    def encode_information(self, s, length=8):
        packet = s.recv(length)
        packet = packet.encode("hex")         # python2
        information = struct.unpack('!d', packet.decode('hex'))[0]
        return information

    def load_calibration_matrix(self):
        d = np.load(os.path.dirname(_root_path)+self._cfg["CALIBRATION_DIR"])
        observed_pts = d['arr_0']
        measured_pts = d['arr_1']
        self._R, self._t = self.get_rigid_transform(observed_pts, measured_pts)

    def get_rigid_transform(self, A, B):
        assert len(A) == len(B)
        N = A.shape[0]  # Total points
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
        BB = B - np.tile(centroid_B, (N, 1))
        H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:  # Special reflection case
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)
        t = np.dot(-R, centroid_A.T) + centroid_B.T
        return R, t

    def uvd2xyz(self, u, v, depth_image, intrinsics):
        fx = intrinsics[0]
        fy = intrinsics[1]
        cx = intrinsics[2]
        cy = intrinsics[3]
        camera_z = np.mean(np.mean(depth_image[v - 5:v + 5, u - 5:u + 5])) / 1000.0
        camera_x = np.multiply(u - cx, camera_z / fx)
        camera_y = np.multiply(v - cy, camera_z / fy)

        view = depth_image[v - 30:v + 30, u - 30:u + 30]
        view[view == 0] = 10000
        avoid_z = np.min(view)

        avoid_v = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[0][0] + v - 5
        avoid_u = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[1][0] + u - 5
        avoid_z = avoid_z / 1000.0
        avoid_x = np.multiply(avoid_u - cx, avoid_z / fx)
        avoid_y = np.multiply(avoid_v - cy, avoid_z / fy)

        xyz = self._R.dot(np.array([camera_x, camera_y, camera_z]).T) + self._t.T
        avoid_xyz = self._R.dot(np.array([avoid_x, avoid_y, avoid_z]).T) + self._t.T
        return list(xyz.T), avoid_xyz[2]


    # def rpy2rotation(self, roll, pitch, yaw):
    #     yawMatrix = np.matrix([
    #         [math.cos(yaw), -math.sin(yaw), 0],
    #         [math.sin(yaw), math.cos(yaw), 0],
    #         [0, 0, 1]
    #     ])
    #
    #     pitchMatrix = np.matrix([
    #         [math.cos(pitch), 0, math.sin(pitch)],
    #         [0, 1, 0],
    #         [-math.sin(pitch), 0, math.cos(pitch)]
    #     ])
    #
    #     rollMatrix = np.matrix([
    #         [1, 0, 0],
    #         [0, math.cos(roll), -math.sin(roll)],
    #         [0, math.sin(roll), math.cos(roll)]
    #     ])
    #
    #     R = yawMatrix * pitchMatrix * rollMatrix
    #     theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    #     multi = 1 / (2 * math.sin(theta))
    #     rx = multi * (R[2, 1] - R[1, 2]) * theta
    #     ry = multi * (R[0, 2] - R[2, 0]) * theta
    #     rz = multi * (R[1, 0] - R[0, 1]) * theta
    #     rotation = np.zeros(3)
    #     rotation[0] = rx
    #     rotation[1] = ry
    #     rotation[2] = rz
    #     return rotation
