import numpy as np
import cv2

class Calibration2D(object):
    def __init__(self):
        self._xy_set = ''
        self._uv_set = ''
        self._M_image2robot = ''
        
    def matrix_update(self):
        pts1 = np.float32(self._uv_set)
        pts2 = np.float32(self._xy_set)
        self._M_image2robot = cv2.getPerspectiveTransform(pts1,pts2)
        print("Matrix updated!")

    def cvt(self, u, v):
        pick_point = [float(u), float(v)]
        grasp_point = np.array([[pick_point]])
        gp_base = cv2.perspectiveTransform(grasp_point, self._M_image2robot)
        x = gp_base[0][0][0]
        y = gp_base[0][0][1]
        return [x, y]
