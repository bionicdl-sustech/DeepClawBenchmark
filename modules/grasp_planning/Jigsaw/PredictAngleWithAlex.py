#predict the pose for grasp
import numpy as np
import os
__currentPath = os.path.dirname(os.path.abspath(__file__))
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(_root_path)
import cv2

# the real method
sys.path.append(_root_path + '/modules/grasp_planning/Jigsaw/AlexNet')
from alexnet import AlexNet
from PredictAngle import *


def normalAngle(angle):
    while(angle >180.0):
        if(angle>180.0):
            angle = angle - 360.0
    return angle

class PredictAngle():
    # Note: reconstructing the __init__ in the child class  will cover the __init__ in the father class
    def loadImg(self,crop_img):
        self.crop_img = crop_img

    def getOrien(self):
        predictor = Grasp_predictor()
        anglePrediceted = predictor.predict_rotation(self.crop_img)
        grasp_angle = normalAngle(anglePrediceted[0])  # -180~180, degree
        return grasp_angle*3.14159/180.0  # radian

    def display(self):
        angle = self.getOrien()
        # self.pose[5] = angle
        return angle


if __name__ == '__main__':
    a = cv2.imread(_root_path+'/data/test_data/ref.jpg')
    pa = PredictAngle()
    pa.loadImg(a)
    angle = pa.display()
    print('angle: ',angle)
