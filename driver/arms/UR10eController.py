# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: UR10eController
@Author: Haokun Wang
@Date: 2020/3/17 15:59
@Description: 
"""
import time
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT)

from driver.arms.ArmController import ArmController
from driver.arms.URConnector import URConnector
from utils.Math import *


class UR10eController(ArmController):
    def __init__(self, configuration_file):
        super(UR10eController, self).__init__()
        self.cfg = configuration_file
        self.__home_pose = self.cfg['HOME_POSE']
        self.__home_joints = self.cfg['HOME_JOINTS']
        self.__connector = URConnector(self.cfg['SOCKET_CONFIGURATION']['robot_ip'],
                                       self.cfg['SOCKET_CONFIGURATION']['port_number'])

    def go_home(self):
        joint = [self.__home_joints[0], self.__home_joints[1], self.__home_joints[2],
                 self.__home_joints[3], self.__home_joints[4], self.__home_joints[5]]
        self.move_j(joint)

    def move_j(self, joints_angle, velocity=0.5, accelerate=0.6,
               solution_space='Joint'):
        joints_angle = [joint * 3.14159 / 180.0 for joint in joints_angle]
        move_command = ""
        if solution_space == 'Joint':
            move_command = (f"movej([{joints_angle[0]},{joints_angle[1]},{joints_angle[2]},"
                            f"{joints_angle[3]},{joints_angle[4]},{joints_angle[5]}],"
                            f"a={accelerate},v={velocity})\n")
        elif solution_space == 'Space':
            move_command = (f"movel([{joints_angle[0]},{joints_angle[1]},{joints_angle[2]},"
                            f"{joints_angle[3]},{joints_angle[4]},{joints_angle[5]}],"
                            f"a={accelerate},v={velocity})\n")

        self.__connector.start()
        self.__connector.send(move_command)
        self.__connector.close()

        return self.verify_state('q_actual', joints_angle)

    def move_p(self, position, velocity=0.5, accelerate=0.6,
               solution_space='Joint'):
        x, y, z = position[0], position[1], position[2]
        rx, ry, rz = rpy2rotation(position[3], position[4], position[5])
        move_command = ""
        if solution_space == 'Joint':
            move_command = f"movej(p[{x},{y},{z},{rx},{ry},{rz}],a={accelerate},v={velocity})\n"
        elif solution_space == 'Space':
            move_command = f"movel(p[{x},{y},{z},{rx},{ry},{rz}],a={accelerate},v={velocity})\n"
        self.__connector.start()
        self.__connector.send(move_command)
        self.__connector.close()

    def get_state(self):
        self.__connector.start()
        msg = self.__connector.ur_get_state('UR10e')
        self.__connector.close()
        return msg

    def verify_state(self, variable_name, target_value, error=0.01, time_out=100):
        flag, count, interval, delta = True, 0, 1, 0
        while flag and count < time_out:
            current_value = self.get_state()[variable_name]
            for c_value, t_value in zip(current_value, target_value):
                if abs(c_value-t_value)>=delta:
                    delta = abs(c_value-t_value)
            if delta < error:
                flag = False
            else:
                time.sleep(interval)
                count = count+1
        if flag:
            print('Time out!')
        return not flag
