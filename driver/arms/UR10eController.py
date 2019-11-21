# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import socket
import struct
import math
import time
import sys
import os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from driver.arms.ArmController import ArmController
from input_output.Configuration import *
from modules.calibration.Calibration3D import *


class UR10eController(ArmController):
    def __init__(self, configuration_path):
        super(UR10eController, self).__init__()
        self._cfg = readConfiguration(configuration_path)
        self._robot_ip = self._cfg['SOCKET_CONFIGURATION']['robot_ip']
        self._port = self._cfg['SOCKET_CONFIGURATION']['port_number']
        self._home_pose = self._cfg['HOME_POSE']
        self._home_joints = self._cfg['HOME_JOINTS']
        self._pick_z = self._cfg['PICK_Z']
        self._place_z = self._cfg['PLACE_Z']
        self._R = None
        self._t = None

        self.load_calibration_matrix()

    def go_home(self):
        joint = [self._home_joints[0], self._home_joints[1], self._home_joints[2],
                 self._home_joints[3], self._home_joints[4], self._home_joints[5]]
        self.move_j(joint)

    def move_j(self, joint, velocity=0.5, accelerate=0.6, solution_space="Joint"):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))

        joint = joint * 3.14159 / 180.0
        if solution_space=="Joint":
            move_command = f"movej([{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}],a={accelerate},v=v{velocity})\n"
        else:
            move_command = f"movel([{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}],a={accelerate},v=v{velocity})\n"
        move_command = bytes(move_command, encoding='utf-8')
        s.send(move_command)
        s.close()

    def move_p(self, position, velocity=0.5, accelerate=0.6, solution_space="Joint"):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))

        if solution_space=="Joint":
            move_command = f"movej([p{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a={accelerate},v=v{velocity})\n"
        else:
            move_command = f"movel([p{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a={accelerate},v=v{velocity})\n"
        move_command = bytes(move_command, encoding='utf-8')
        s.send(move_command)
        s.close()

    def set_io(self, port, value):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        command = f"set_digital_out({port}, {value})\n"
        command = bytes(command, encoding='utf-8')
        s.send(command)
        s.close()

    def get_state(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        command = f"get_state()\n"
        command = bytes(command, encoding='utf-8')
        s.send(command)

        packet = s.recv(444)
        x = self.encode_information(s)
        y = self.encode_information(s)
        z = self.encode_information(s)
        Rx = self.encode_information(s)
        Ry = self.encode_information(s)
        Rz = self.encode_information(s)
        beta = (1 - 2 * 3.14 / math.sqrt(Rx * Rx + Ry * Ry + Rz * Rz))
        Rx = Rx * beta
        Ry = Ry * beta
        Rz = Rz * beta

        return {"Position": [x, y, z, Rx, Ry, Rz]}

    def verify_state(self, variable_name, target_value, error=0.2):
        cnt = 0
        delay_flag = True
        time_gap = 0.2
        while delay_flag:
            current_value = self.get_state()[variable_name]
            if current_value is None:
                print("Getting current value failed, please check variable name.")
                return False
            if cnt * time_gap >= 20:
                print("Time out!")
                return False
            for c_value, t_value in zip(current_value, target_value):
                if abs(c_value - t_value) > error:
                    delay_flag = True
                    break
                else:
                    delay_flag = False
            time.sleep(time_gap)
            cnt += 1
        return True

    def encode_information(self, s, length=8):
        packet = s.recv(length)
        packet = packet.encode("hex")
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
