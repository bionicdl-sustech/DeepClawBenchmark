import numpy as np


class ProgressiveMoving:
    def __init__(self, threshold=0.05, factor=0.5):
        self.threshold = threshold
        self.factor = factor

    def display(self, current_pose, target_pose):
        c_pose = np.array(current_pose)
        t_pose = np.array(target_pose)

        if length >= self.threshold:
            direction_vector = (t_pose - c_pose) * self.factor
            t_pose = c_pose + direction_vector
        return list(t_pose)