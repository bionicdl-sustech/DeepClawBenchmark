import sys
sys.path.append('/home/yang/python-urx')
sys.path.append('/home/yang/Git/CobotBenchmark/Driver')
sys.path.append('/home/yang/Git/CobotBenchmark/Functions')
sys.path.append('/home/yang/Git/CobotBenchmark/ToolKit')
from fc_predictor import Predictor
from realsense_controller import RealsenseController
import cv2
from PIL import Image, ImageDraw
import time
import numpy as np
import ur_controller_urx as urc
import Calibration_2D as Cali
from datetime import datetime
from success_label import *
import random
from random import choice
import os
import heapq

WorkSpace = [-0.21,-0.37,0.21,-0.68]
rob = urc.ur5()
camera = RealsenseController()
hand_eye=Cali.calibration()
crop_box = [340,160,980,700]
str = '/media/yang/Linux/Data/OriginalData/2-Fingers_Guided'
G = Predictor('/home/yang/Git/CobotBenchmark/Functions/checkpoint/fc_cnn(new)/Toys') # /ur5
color_image,depth_image,infrared_L,infrared_R = camera.getImage()
rob.homing()
rob.close_gripper()
with open(str+'/TrainingRecord', 'aw') as f:
    f.write('i,x,y,rz,success_label'+'\n')

for i in range(999):

    color_image1,depth_image,infrared_L,infrared_R = camera.getImage()
    if not os.path.exists(str+"/"+"TrialCount_%01d"%i):
        os.makedirs(str+"/"+"TrialCount_%01d"%i)
    strnew=str+"/"+"TrialCount_%01d"%i
    cv2.imwrite(strnew +"/"+"image_01.jpg", color_image1)
    img = Image.open(strnew+'/image_01.jpg').crop(crop_box)
    img.save(strnew +"/"+"Croppedimage.jpg")

    rob.open_gripper()

    image = np.array(img).reshape(1,(crop_box[3]-crop_box[1]),(crop_box[2]-crop_box[0]),3)
    y_value = G.eval(image,(crop_box[3]-crop_box[1]),(crop_box[2]-crop_box[0]))
    angle_patch,probability_patch = G.parse_eval(y_value)
    probability_patch_sub = []
    for j in range(len(probability_patch)):
        if probability_patch[j] > 0.5:
            probability_patch_sub.append(j)
    if len(probability_patch_sub) == 0:
        max_num_list = map(probability_patch.index, heapq.nlargest(5, probability_patch))
        b = random.sample(max_num_list,1)
        idx = b[0]
    else:
        idx = choice(probability_patch_sub)
    y = (int(idx/y_value.shape[2]))*((crop_box[3]-crop_box[1]-238)/(y_value.shape[1]-1))+111+crop_box[1]
    x = (idx%y_value.shape[2])*((crop_box[2]-crop_box[0]-238)/(y_value.shape[2]-1))+111+crop_box[0]
    rz = (-1.57 + (random.sample(range(18),1)[0]+0.5)*(1.57/9))
    print(x,y,rz)
    x,y = hand_eye.cam2rob(x,y)
    rob.move([x,y,0.25],rz)
    rob.grasping()
    rob.homing()
    time.sleep(5)

    x2=random.uniform(WorkSpace[0]+0.1,WorkSpace[2]-0.1)
    y2=random.uniform(WorkSpace[1]-0.1,WorkSpace[3]+0.1)
    rz2=random.uniform(-1.57,1.57)
    color_image2,depth_image,infrared_L,infrared_R = camera.getImage()
    cv2.imwrite(strnew +"/"+"image_02.jpg", color_image2)
    grasp_label = success_label(color_image1,color_image2)

    if grasp_label == 1:

        rob.move([x2, y2, 0.20],rz2)
        rob.move([x2, y2, 0.15],rz2)

        rob.open_gripper()
        rob.homing()
        rob.close_gripper()

    with open(str+'/TrainingRecord', 'aw') as f:
        f.write("%d,"%i+"%f,"%x+"%f,"%y+"%f,"%rz+"%d"%grasp_label+'\n')
