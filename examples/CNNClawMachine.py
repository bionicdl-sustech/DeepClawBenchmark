import os
import sys
import time
from PIL import Image, ImageDraw
import numpy as np
_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)
import heapq
import random
from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor
from modules.end2end.grasp_alexnet.Predictor import Predictor
from input_output.observers.ResultMonitor import ResultMonitor
from modules.success_label.success_label import success_label
from modules.localization.random_seg import RandomSeg
from modules.grasp_planning.random_planner import RandomPlanner

NUM_BOXES = 15
WIDTH = 120
crop_box = [480,200,850,600]
class CNNClawMachine(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(CNNClawMachine, self).__init__(perception_system, manipulation_system, is_debug)
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
        self.network = Predictor(_root_path+"/data/checkpoint/Toys")
        self.motion_planner = ''
        self.arm.load_calibration_matrix()
        self.localization_operator_pick = RandomSeg([[-0.2,0.2], [-0.76, -0.46], [0.25, 0.25]])
        self.localization_operator_place = RandomSeg([[-0.1,0.1], [-0.6, -0.6], [0.50, 0.50]])
        self.recognition_operator = ''
        self.grasp_planner = RandomPlanner([[3.14, 3.14],[0, 0],[-1.57, 1.57]])
        self.motion_planner = ''
        self.path=''
        self.Pick_z = 0.33
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

        # end2end prediction
        self.arm.set_io(8,True)
        time.sleep(3.5)
        frame_1= self.camera.get_frame()
        color_image_1 = frame_1.color_image[0]
        self.arm.set_io(8,False)
        time.sleep(2)

        start = time.time()
        img = Image.fromarray(np.uint8(color_image_1))
        img = img.crop(crop_box)
        patches, boxes = self.network.generate_patches(img, NUM_BOXES, WIDTH)
        candidates_theta, candidates_probability = self.network.eval_theta(patches)
        # best_idx = np.argmax(candidates_probability)
        candidates_probability_1 = list(candidates_probability)
        idx = map(candidates_probability_1.index, heapq.nlargest(3,candidates_probability_1))
        best_idx = random.sample(idx,1)
        best_idx = best_idx[0]
        end = time.time()

        #publish dataget_frame
        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)
        self.image_monitor.img_name = self.experiment_name + '_image1.png'
        idata = {'Image':color_image_1}
        self.publisher.sendData(idata)

        #draw picture
        draw = ImageDraw.Draw(img, 'RGBA')
        for j in range(len(boxes)):
            x = (boxes[j][0]+boxes[j][2])/2
            y = (boxes[j][1]+boxes[j][3])/2
            r = candidates_probability[j] * ((img.size[0]-WIDTH)/NUM_BOXES/2)

            if j == best_idx or candidates_probability[j]>0.1:
                draw.ellipse((x-r, y-r, x+r, y+r), (0, 0, 255, 125))
                best_theta = (-1.57 + (candidates_theta[j]+0.5)*(1.57/9))
                draw.line([(x-r*np.cos(best_theta),y+r*np.sin(best_theta)),
                       (x+r*np.cos(best_theta), y-r*np.sin(best_theta))], fill=(255,255,255,125), width=10)
        img.save(self.path+'result_1.png')
        x = (boxes[best_idx][0]+boxes[best_idx][2])/2+crop_box[0]
        y = (boxes[best_idx][1]+boxes[best_idx][3])/2+crop_box[1]
        best_theta = ( (candidates_theta[best_idx]-0.5)*(1.57/9))
        # best_theta = -0.78
        # motion planning
        start = time.time()
        position,z = self.arm.uvd2xyz(x,y,frame_1.depth_image[0],self.camera.get_intrinsics())
        grasp_pose = [position[0],position[1],self.Pick_z,-3.14,0,best_theta]
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
        compare_space=[0,100,500,750]
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
        grasp_pose = [0,-0.6,0.5,-3.14,0,0]
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
