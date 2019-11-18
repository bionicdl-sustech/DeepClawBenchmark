import os
import sys
import numpy as np
from scipy.linalg import eigh

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.grasp_planning.grasp_planning import GraspPlaner


class PrincipalAxisPlanner(GraspPlaner):
    def __init__(self):
        pass

    def display(self, point_cloud, centers, labels, **kwargs):
        thetas = []
        confidences = []

        for i in range(len(set(labels)) - 1):
            x = point_cloud[labels == i][:, :2]
            x = x - np.mean(x, axis=0)
            xx = np.dot(x.T, x)
            w, v = eigh(xx)
            confidences.append(np.abs(w[-1]) / np.abs(w[0]))

            eigv = v[-1]
            # center = np.mean(x, axis=0)
            thetas.append(np.arctan2(eigv[1], eigv[0]) % np.pi)

        index = np.where(confidences == np.max(confidences))[0]

        return [centers[index][0], centers[index][1], 0.05, thetas[index], 0, 0]
