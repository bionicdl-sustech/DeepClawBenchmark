import cv2
import sys
import numpy as np
from matplotlib import pyplot as plt
import time
import DetectForeground as df
import os
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
from driver.sensors.camera.RealsenseController import RealsenseController
def success_label(img1,img2,compare_space=[630,830,50,100]):


    img1 = img1[compare_space[0]:compare_space[1],compare_space[2]:compare_space[3]]



    img2 = img2[compare_space[0]:compare_space[1],compare_space[2]:compare_space[3]]

    compare = df.Segment()
    rect = compare.DiffGround(img1,img2)
    if len(rect) > 0 :
        success_label = 1
        print('success_label:'+str(success_label))
        return success_label,img1,img2
    else :
        success_label = 0
        print('success_label:'+str(success_label))

        return success_label,img1,img2

    # print(rect[0])
    #
    # imag = cv2.rectangle(imagegray,(rect[0][0],rect[0][1]),(rect[0][0]+rect[0][2],rect[0][1]+rect[0][3]),(0,255,0),3)
    # cv2.imshow('img',imag)
    # cv2.waitKey()

    # print(rect)
if __name__=="__main__":

    realsense = RealsenseController("/config/sensors/realsense.yaml")
    img = realsense.get_frame()
    print(img.color_image[0].shape)
    cv2.imwrite(_root_path, img.color_image[0])
    img1 = img.color_image[0][630,830,50,100]
    cv2.imwrite('img1', img1)
