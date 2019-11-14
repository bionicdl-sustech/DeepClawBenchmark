import os
import sys
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.grasp_planning.grasp_planning import GraspPlaner


class RandomPlanner(GraspPlaner):
    def __init__(self, rpy_constrain):
        self.min_roll = rpy_constrain[0][0]
        self.max_roll = rpy_constrain[0][1]
        self.min_pitch = rpy_constrain[1][0]
        self.max_pitch = rpy_constrain[1][1]
        self.min_yaw = rpy_constrain[2][0]
        self.max_yaw = rpy_constrain[2][1]

    def display(self, centers, **kwargs):
        index = np.random.uniform(0, len(centers))
        x, y, z = centers[index][0], centers[index][1], centers[index][2]
        roll = np.random.uniform(self.min_roll, self.max_roll)
        pitch = np.random.uniform(self.min_pitch, self.max_pitch)
        yaw = np.random.uniform(self.min_yaw, self.max_yaw)
        return [x, y, z, roll, pitch, yaw]
