import os
import sys
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.grasp_planning.grasp_planning import GraspPlaner


class RandomPlanner(GraspPlaner):
    def __init__(self, xyz_constrain, rpy_constrain):
        self.min_x = xyz_constrain[0][0]
        self.max_x = xyz_constrain[0][1]
        self.min_y = xyz_constrain[1][0]
        self.max_y = xyz_constrain[1][1]
        self.min_z = xyz_constrain[2][0]
        self.max_z = xyz_constrain[2][1]
        self.min_roll = rpy_constrain[0][0]
        self.max_roll = rpy_constrain[0][1]
        self.min_pitch = rpy_constrain[1][0]
        self.max_pitch = rpy_constrain[1][1]
        self.min_yaw = rpy_constrain[2][0]
        self.max_yaw = rpy_constrain[2][1]

    def display(self, color_image=None, depth_image=None, point_cloud=None,
                labels=None, probability=None):
        x = np.random.uniform(self.min_x, self.max_x)
        y = np.random.uniform(self.min_y, self.max_y)
        z = np.random.uniform(self.min_z, self.max_z)
        roll = np.random.uniform(self.min_roll, self.max_roll)
        pitch = np.random.uniform(self.min_pitch, self.max_pitch)
        yaw = np.random.uniform(self.min_yaw, self.max_yaw)
        return [x, y, z, roll, pitch, yaw]
