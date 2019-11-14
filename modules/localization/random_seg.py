import os
import sys
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.localization.localization import Localization


class RandomSeg(Localization):
    def __init__(self, xyz_constrain):
        self.min_x = xyz_constrain[0][0]
        self.max_x = xyz_constrain[0][1]
        self.min_y = xyz_constrain[1][0]
        self.max_y = xyz_constrain[1][1]
        self.min_z = xyz_constrain[2][0]
        self.max_z = xyz_constrain[2][1]

    def display(self, **kwargs):
        x = np.random.uniform(self.min_x, self.max_x)
        y = np.random.uniform(self.min_y, self.max_y)
        z = np.random.uniform(self.min_z, self.max_z)
        centers = [[x, y, z]]
        return None, None, centers
