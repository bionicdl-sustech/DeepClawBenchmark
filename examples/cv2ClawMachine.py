import os
import sys
import time
from PIL import Image, ImageDraw
import numpy as np
import cv2
_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor
from input_output.observers.ResultMonitor import ResultMonitor
from modules.success_label.success_label import success_label
from modules.localization.random_seg import RandomSeg
from modules.grasp_planning.random_planner import RandomPlanner
from modules.localization.contour_filter import ContourFilter

crop_box = [200,550,450,850]
class cv2ClawMachine(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(cv2ClawMachine, self).__init__(perception_system, manipulation_system, is_debug)
        time_s = time.localtime(int(time.time()))
        self.experiment_name = "experiment_" + str(time_s.tm_mon) + str(time_s.tm_mday) + \
                               str(time_s.tm_hour) + str(time_s.tm_min) + str(time_s.tm_sec)
        self.data_path = _root_path + "/data/" + os.path.basename(__file__) + "/" + self.experiment_name + "/"
        self.args = {}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.result_monitor = ResultMonitor("result_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)
        self.publisher.registerObserver(self.result_monitor)
        self.motion_planner = ''
        self.arm.load_calibration_matrix()
        self.localization_operator_pick = RandomSeg([[-0.2,0.2], [-0.76, -0.46], [0.25, 0.25]])
        self.localization_operator_place = RandomSeg([[-0.1,0.1], [-0.6, -0.6], [0.05, 0.05]])
        self.recognition_operator = ''
        self.grasp_planner = RandomPlanner([[3.14, 3.14],[0, 0],[-1.57, 1.57]])
        self.motion_planner = ''
        self.path=''
        self.Pick_z = 0.025
        self.uv_range = [7, 11]
    def task_display(self):
        # initialize robots
        self.arm.set_io(8,False)
        self.arm.go_home()
        self.arm.set_io(8,False)
        #initialize camera
        frame= self.camera.get_frame()

        #result Recorder
        self.result_monitor.dir = self.data_path
        self.result_monitor.csv_name = self.experiment_name+".csv"
        count_success = 0
        # sub-task display
        for i in range(10):
            self.args["subtask_counter"] = i
            # subtask data path
            subtask_name = "subtask_pick_" + str(self.args["subtask_counter"])
            self.path = self.data_path + subtask_name + "/"
            self.time_monitor.dir = self.path
            self.time_monitor.csv_name = self.experiment_name+".csv"
            self.image_monitor.dir = self.path

            compare_label = self.subtask_display_pick()
            if compare_label == 1:
                sdata = {"Result": [subtask_name,compare_label]}
                self.publisher.sendData(sdata)
                self.subtask_display_place()
                count_success = count_success+1
            if compare_label == 0:
                sdata = {"Result": [subtask_name,compare_label]}
                self.publisher.sendData(sdata)
            sdata = {"Result": ['count_success',count_success]}
            self.publisher.sendData(sdata)
            print(count_success)
    def subtask_display_pick(self):
        # subtask data path
        subtask_name = "subtask_pick_" + str(self.args["subtask_counter"])
        self.path = self.data_path + subtask_name + "/"

        # localization
        self.arm.set_io(8,True)
        time.sleep(3.5)
        frame_1= self.camera.get_frame()
        color_image_1 = frame_1.color_image[0]
        self.arm.set_io(8,False)
        time.sleep(2)

        start = time.time()
        img = color_image_1 [crop_box[0]:crop_box[1],crop_box[2]:crop_box[3]]
        contour_filter = ContourFilter(area_threshold = [50,150000],minAreaBox = True)
        bounding_boxes, mask, centers = contour_filter.display(img)
        end = time.time()
        a = []
        for i in range(len(bounding_boxes)):
            a.append(bounding_boxes[i][1][0]*bounding_boxes[i][1][1])
            # cv2.rectangle(img,(bounding_boxes[i][0][0],bounding_boxes[i][0][1]),(bounding_boxes[i][0][0]+bounding_boxes[i][1][0],bounding_boxes[i][0][1]+bounding_boxes[i][1][1]),(0,255,0),3)
        index = a.index(max(a))
        box = cv2.boxPoints(bounding_boxes[index])

        #draw picture
        box_d = np.int0(box)
        cv2.drawContours(img, [box_d], 0, (0,255,0), 3)
        cv2.rectangle(img,(centers[index][0],centers[index][1]),(centers[index][0],centers[index][1]),(0,255,0),3)
        x = centers[index][0]
        y = centers[index][1]
        best_theta = bounding_boxes[index][2]
        print(x,y,best_theta)
        #publish data
        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)
        self.image_monitor.img_name = self.experiment_name + '_image1.png'
        idata = {'Image':color_image_1}
        self.publisher.sendData(idata)
        self.image_monitor.img_name = self.experiment_name + '_image1_1.png'
        idata = {'Image':img}
        self.publisher.sendData(idata)

        # motion planning
        start = time.time()
        position,z = self.arm.uvd2xyz(x+crop_box[2],y+crop_box[0],frame_1.depth_image[0],self.camera.get_intrinsics())
        grasp_pose = [position[0],position[1],self.Pick_z,-3.14,0,best_theta*3.14/180]
        print(grasp_pose)
        #pick
        grasp_pose[2] = grasp_pose[2]+0.1
        self.arm.move_p(grasp_pose)
        grasp_pose[2] = grasp_pose[2]-0.1
        self.arm.move_p(grasp_pose)
        self.arm.set_io(8,True)
        time.sleep(2)
        grasp_pose[2] = grasp_pose[2]+0.1
        self.arm.move_p(grasp_pose)
        self.arm.go_home()
        end = time.time()

        #verify success
        frame_2= self.camera.get_frame()
        color_image_2 = frame_2.color_image[0]
        compare_space=[50,200,630,830]
        compare_label,img1,img2 = success_label(color_image_1,color_image_2,compare_space)

        #publish data
        tdata = {"Time": [subtask_name + ' execution_time', end - start]}
        self.publisher.sendData(tdata)
        self.image_monitor.img_name = self.experiment_name + '_image2.png'
        idata = {'Image':color_image_2}
        self.publisher.sendData(idata)
        self.image_monitor.img_name = self.experiment_name + '_image3.png'
        idata = {'Image':img1}
        self.publisher.sendData(idata)
        self.image_monitor.img_name = self.experiment_name + '_image4.png'
        idata = {'Image':img2}
        self.publisher.sendData(idata)

        return compare_label

    def subtask_display_place(self):
        print('subtask_display_place')
        subtask_name = "subtask_place_" + str(self.args["subtask_counter"])

        # segmentation
        print('segmentation')
        start = time.time()
        bounding_box, mask, centers = self.localization_operator_place.display()
        end = time.time()

        tdata = {"Time": [subtask_name+' segmentation_time', end - start]}
        self.publisher.sendData(tdata)

        # recognition
        print('recognition')
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        print('grasp planning')
        start = time.time()
        grasp_pose = self.grasp_planner.display(centers)
        end = time.time()

        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # motion planning
        print('motion planning')
        start = time.time()
        self.arm.move_p(grasp_pose)
        self.arm.set_io(8,False)
        time.sleep(2)
        self.arm.go_home()
        end = time.time()

        tdata = {"Time": [subtask_name + ' motion_planning_time', end - start]}
        self.publisher.sendData(tdata)
