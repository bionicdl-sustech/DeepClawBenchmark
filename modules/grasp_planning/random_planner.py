import os
import sys
import random
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

    def display(self, centers, labels, **kwargs):
        if labels is not None:
            candidated_centers = []
            for center, label in zip(centers, labels):
                if label == 1:
                    candidated_centers.append(center)
            
            if len(candidated_centers) != 0:
                index = random.randint(0, len(candidated_centers)-1)
                if len(candidated_centers[0]) == 3:
                    u, v, z = candidated_centers[index][0], candidated_centers[index][1], candidated_centers[index][2]
                else:
                    u, v, z = candidated_centers[index][0], candidated_centers[index][1], 0
                roll = np.random.uniform(self.min_roll, self.max_roll)
                pitch = np.random.uniform(self.min_pitch, self.max_pitch)
                yaw = np.random.uniform(self.min_yaw, self.max_yaw)
                return [u, v, z, roll, pitch, yaw]
            else:
                return None
