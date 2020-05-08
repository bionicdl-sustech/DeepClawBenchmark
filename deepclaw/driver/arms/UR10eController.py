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

import yaml
from scipy.spatial.transform import Rotation as RR

class UR10eController(ArmController):
    def __init__(self, robot_configuration_file_path):
        super(UR10eController, self).__init__()
        self._cfg = yaml.load(open(robot_configuration_file_path,'r'),Loader=yaml.FullLoader)
        self._home_joints = self._cfg['home_joint']
        self._home_pose = self._cfg['home_pose']
        self._ip = self._cfg['ip']
        self._port = self._cfg['port']
        self._v = self._cfg['velocity']
        self._a = self._cfg['acceleration']

        # create connection
        self._connector = URConnector(self._ip,self._port)

    def go_home(self):
        joint = [self._home_joints[0], self._home_joints[1], self._home_joints[2],
                 self._home_joints[3], self._home_joints[4], self._home_joints[5]]
        self.move_j(joint)

    '''
    @Description: go to the target joints positions
    @param[in]: joints_angle,target joint positions  of each joints;
                velocity:joint acceleration of leading axis [rad/s]
                accelerate: joint speed of leading axis [rad/^2]
                solution_space: move style, 'Joint' means it linear in joint-space,
                                and 'Space' means linear in tool-space(forward kinematics is used to calculate the corresponding pose)

    @return: bool, reaching target or not
    '''
    def move_j(self, joints_angle, velocity=None, acceleration=None,
               solution_space='Joint'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration

        move_command = ""
        if solution_space == 'Joint':
            move_command = (f"movej([{joints_angle[0]},{joints_angle[1]},{joints_angle[2]},"
                            f"{joints_angle[3]},{joints_angle[4]},{joints_angle[5]}],"
                            f"a={acceleration},v={velocity})\n")
        elif solution_space == 'Space':
            move_command = (f"movel([{joints_angle[0]},{joints_angle[1]},{joints_angle[2]},"
                            f"{joints_angle[3]},{joints_angle[4]},{joints_angle[5]}],"
                            f"a={acceleration},v={velocity})\n")

        self._connector.start()
        self._connector.send(move_command)
        self._connector.close()
        return self.verify_state('q_actual', joints_angle)

    '''
    @Description: go to the target pose
    @param[in]: position,target pose [x,y,z,r,p,y];
                velocity:joint acceleration of leading axis [rad/s]
                accelerate: joint speed of leading axis [rad/s^2]
                solution_space: move style, 'Joint' means it linear in joint-space(inverse kinematics is used to calculate the corresponding joints),
                                and 'Space' means linear in tool-space

    @return: bool, reaching target or not
    '''
    def move_p(self, position, velocity=None, acceleration=None,
               solution_space='Joint'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration

        x, y, z = position[0], position[1], position[2]
        # temp = RR.from_euler('xyz', [position[3], position[4], position[5]], degrees=False)
        # rx, ry, rz = temp.as_rotvec()
        rx, ry, rz = position[3], position[4], position[5]

        move_command = ""
        if solution_space == 'Joint':
            move_command = f"movej(p[{x},{y},{z},{rx},{ry},{rz}],a={acceleration},v={velocity})\n"
        elif solution_space == 'Space':
            move_command = f"movel(p[{x},{y},{z},{rx},{ry},{rz}],a={acceleration},v={velocity})\n"

        self._connector.start()
        self._connector.send(move_command)
        self._connector.close()
        return self.verify_state('tool_vector_actual', [x, y, z, rx, ry, rz])

    def move_ps(self, positions, velocity=None, acceleration=None, solution_space='Joint'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration

        move_command = f"def process():\n"
        move_command += f" movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.1)\n" % (positions[0][0], positions[0][1], positions[0][2], positions[0][3], positions[0][4], positions[0][5], acceleration, velocity)
        move_command += f" movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (positions[1][0], positions[1][1], positions[1][2], positions[1][3], positions[1][4], positions[1][5], acceleration, velocity)
        move_command += f"end\n"
        # print(move_command)

        self._connector.start()
        self._connector.send(move_command)
        self._connector.close()
        return self.verify_state('tool_vector_actual', positions[1])

    # get robot state
    def get_state(self):
        self._connector.start()
        msg = self._connector.ur_get_state('UR10e')
        self._connector.close()
        return msg

    '''
    @Description: verify the robot reaching the target pose(joint or cartesian) or not
    @param[in]:
    variable_name: target style, joints('q_actual') or cartesian('tool_vector_actual')
    target_value: target values
    error: threshold,if the difference between current state and target state is small than threshold, we say the robot reached the target.
    time_out: Max time of the motion, unit: second
    '''
    def verify_state(self, variable_name, target_value, error=0.0001, time_out=10):
        # flag: bool, reaching target or not
        t1 = time.time()
        flag = True
        while flag:
            current_value = self.get_state()[variable_name]
            delta = 0
            for c_value, t_value in zip(current_value, target_value):
                if abs(c_value-t_value)>=delta:
                    delta = abs(c_value-t_value)
            t2 = time.time()
            dt = t2-t1
            # reach the target or time out break the loop
            if delta < error:
                flag = False
            elif dt > time_out:
                break
            else:
                continue
        if flag:
            print('--This motion need more time--!')
        return not flag

if __name__ == '__main__':
    robot  = UR10eController('../../../configs/robcell-ur10e-hande-d435/ur10e.yaml')
    robot.go_home()
    print('reach home pose')
    robot.move_j([-1.26675971, -1.50360084, -2.01986912, -1.18507832,  1.55369178,1.2],0.8,1.2)
    robot.move_p([0.01599,-0.63382,0.56354,0, -3.14, 0])
    robot.go_home()
    state = robot.get_state()
    print(state)
