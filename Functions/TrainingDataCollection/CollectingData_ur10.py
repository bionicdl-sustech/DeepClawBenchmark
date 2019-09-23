import numpy as np
import random
import sys
import cv2
import os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from Driver.Camera.RealsenseController import RealsenseController
from Driver.UR10e.UrController import URController
import ToolKit.success_label as label


realsense = RealsenseController()

rob = URController()
rob.openGripper()
rob.goHome()

str = '/home/yang/DeepClaw/Training/CollectingData/Data/ur10_stage8'
for i in range(442,1000):
    '''
    take pictures and save
    '''
    color_image, _ = realsense.getImage()
    if not os.path.exists(str+"/"+"TrialCount_%01d"%i):
        os.makedirs(str+"/"+"TrialCount_%01d"%i)
    strnew=str+"/"+"TrialCount_%01d"%i
    np.save(strnew +"/"+"image_D_01", depth_image)
    cv2.imwrite(strnew +"/"+"image_C_01.jpg", color_image)

    '''
    Blind grasp
    '''

    x=random.uniform(0.103,0.594)
    y=random.uniform(0.734,0.346)
    yy=random.uniform(-1.57,1.57)
    print(x,y,yy)

    rob.move([x,y,0.4],yy)
    rob.grasping()

    # '''
    # take pictures and save
    # '''
    #
    # color_image,depth_image,infrared_L,infrared_R = getImageFromRealsense()
    # np.save(strnew +"/"+"image_D_02", depth_image)
    # cv2.imwrite(strnew +"/"+"image_C_02.jpg", color_image)

    '''
    record the result and position
    '''
    rob.homing()
    success_label,imagegray= label.success_label()
    '''
    take pictures and save
    '''

    color_image,depth_image,infrared_L,infrared_R = getImageFromRealsense()
    np.save(strnew +"/"+"image_D_02", depth_image)
    cv2.imwrite(strnew +"/"+"image_C_02.jpg", color_image)
    cv2.imwrite(strnew +"/"+"image_C_03.jpg", imagegray)
    with open(str+'/TrainingRecord', 'aw') as f:
        f.write("%d,"%i+"%f,"%x+"%f,"%y+"0,"+"-3.14,"+"0,"+"%f,"%yy+"%d"%success_label+'\n')

    '''
    place on the other place
    '''
    x=random.uniform(0.15,0.55)
    y=random.uniform(0.4,0.7)
    yy=random.uniform(-1.57,1.57)
    print(x,y,yy)

    rob.move([x,y,0.4],yy)
    rob.open_gripper()
    rob.homing()
