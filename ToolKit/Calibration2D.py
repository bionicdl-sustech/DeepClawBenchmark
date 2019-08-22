import numpy as np
import cv2

class Calibration2D(object):
    def __init__(self):
        xy_set = ''

        uv_set = ''
        self.M_image2robot = ''
        
    def matrix_update(self):
        pts1 = np.float32(self.uv_set)
        pts2 = np.float32(self.xy_set)
        self.M_image2robot = cv2.getPerspectiveTransform(pts1,pts2)
        print("Matrix updated!")

    def cvt(self, u, v):
        pick_point = [float(u), float(v)]
        grasp_point = np.array([[pick_point]])
        gp_base = cv2.perspectiveTransform(grasp_point, self.M_image2robot)
        x = gp_base[0][0][0]
        y = gp_base[0][0][1]
        return [x, y]
