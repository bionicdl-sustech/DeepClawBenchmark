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

        # Default joint speed configuration
        self.joint_acc = self._cfg['JOINT_ACC']
        self.joint_vel = self._cfg['JOINT_VEL']

        # Joint tolerance in radian for blocking program
        self.joint_tolerance = [0.01,0.01,0.01,0.01,0.01,0.01]

        # Default tool speed configuration
        self.tool_acc = 0.5 # Safe: 0.5
        self.tool_vel = 0.2 # Safe: 0.2

        # Tool pose tolerance for blocking program
        self.tool_pose_tolerance = [0.001,0.001,0.001,0.01,0.01,0.01]

        # self.load_calibration_matrix()

    def go_home(self):
        joint = [self._home_joints[0], self._home_joints[1], self._home_joints[2],
                 self._home_joints[3], self._home_joints[4], self._home_joints[5]]
        self.move_j(joint)

    def move_j(self, joint, solution_space="Joint"):
        """
        parameter joint is in degree.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))

        if solution_space == "Joint":
            # move_command = f"movej([{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movej([{},{},{},{},{},{}],a={},v={})\n".format(joint[0] * 3.14159 / 180.0,
                                                                           joint[1] * 3.14159 / 180.0,
                                                                           joint[2] * 3.14159 / 180.0,
                                                                           joint[3] * 3.14159 / 180.0,
                                                                           joint[4] * 3.14159 / 180.0,
                                                                           joint[5] * 3.14159 / 180.0,
                                                                           self.joint_acc, self.joint_vel)
        else:
            # move_command = f"movel([{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movel([{},{},{},{},{},{}],a={},v={})\n".format(joint[0], joint[1], joint[2],
                                                                           joint[3], joint[4], joint[5],
                                                                           self.tool_acc, self.tool_vel)
        # move_command = bytes(move_command, encoding='utf-8')
        s.send(move_command)
        s.close()
        self.verify_state("Joint", joint)

    def move_p(self, position, solution_space="Joint"):
        """
        parameter position is (x, y, x, roll, pitch, yaw) in meter and radian.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        Rx, Ry, Rz = self.rpy2rotation(position[3], position[4], position[5])

        if solution_space == "Joint":
            # move_command = f"movej([p{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movej(p[{},{},{},{},{},{}],a={},v={})\n".format(position[0], position[1], position[2],
                                                                            Rx, Ry, Rz,
                                                                            self.joint_acc, self.joint_vel)
        else:
            # move_command = f"movel([p{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a={accelerate},v=v{velocity})\n"
            move_command = "movel(p[{},{},{},{},{},{}],a={},v={})\n".format(position[0], position[1], position[2],
                                                                            Rx, Ry, Rz,
                                                                            self.tool_acc, self.tool_vel)
        # move_command = bytes(move_command, encoding='utf-8')
        s.send(move_command)
        s.close()
        self.verify_state("Position", position[:3]+[Rx, Ry, Rz])

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

        return {"Position": [x, y, z, Rx, Ry, Rz], "Joint": [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]}

    def verify_state(self, variable_name, target_value, FT = False):
        delay_time = True
        cnt = 0
        timeGap = 0.01

        if variable_name == "Joint":
            error = self.joint_tolerance
            while(delay_time and cnt < 1000):
                currentStatus = self.get_state()
                currentPose = currentStatus[variable_name]
                if currentPose is None:
                    print("Getting current value failed, please check variable name.")
                    return False
                dpose = abs(np.array(currentPose) - np.array(target_value))/180*3.1415
                if(FT == True):
                    # import ft mudules
                    from driver.sensors.FT.getFT_Data import detectCollision
                    collosion_bool = detectCollision()
                    if collosion_bool==True:
                        return False
                    else:
                        pass
                if (all( [dpose[j] < error[j] for j in range(6)] )):
                    delay_time = False
                    return True
                time.sleep(timeGap)
                cnt = cnt + 1
            print("Time Out 10 seconds!")
            return False
        else:
            error = self.tool_pose_tolerance
            while(delay_time and cnt < 1000):
                currentStatus = self.get_state()
                currentPose = currentStatus[variable_name]
                if currentPose is None:
                    print("Getting current value failed, please check variable name.")
                    return False
                dpose = abs(np.array(currentPose) - np.array(target_value))
                for a in [3,4,5]:
                    if dpose[a]>6:
                        dpose[a] = dpose[a] - 2* 3.141592653589793
                    if dpose[3]<-6:
                        dpose[a] = dpose[a] + 2* 3.141592653589793
                if(FT == True):
                    # import ft mudules
                    from driver.sensors.FT.getFT_Data import detectCollision
                    collosion_bool = detectCollision()
                    if collosion_bool==True:
                        return False
                    else:
                        pass
                if (all( [dpose[j] < error[j] for j in range(6)] )):
                    delay_time = False
                    return True
                time.sleep(timeGap)
                cnt = cnt + 1
            print("Time Out 10 seconds!")
            return False


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
