import cv2
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor
from input_output.observers.TicTacToeDataMonitor import TicTacToeDataMonitor
from modules.localization.contour_filter import contour_filter
from modules.recognition.color_recognition import color_recognition
from modules.grasp_planning.random_planner import RandomPlanner


class TicTacToe(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(TicTacToe, self).__init__(perception_system, manipulation_system, is_debug)
        time_s = time.localtime(int(time.time()))
        self.experiment_name = "experiment_" + str(time_s.tm_mon) + str(time_s.tm_mday) + \
                               str(time_s.tm_hour) + str(time_s.tm_min) + str(time_s.tm_sec)
        self.data_path = _root_path+"/data/"+os.path.basename(__file__)+"/"+self.experiment_name+"/"
        self.args = {"WorkSpace": [[350, 550], [450, 850]]}
        # self.args = {"WorkSpace": [[100, 550], [450, 850]]}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.data_monitor = TicTacToeDataMonitor("data_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)
        self.publisher.registerObserver(self.data_monitor)

        self.localization_operator = contour_filter(area_threshold=[300, 500])
        # self.localization_operator = contour_filter(area_threshold=[900, 1050])
        self.recognition_operator = color_recognition([100, 43, 46], [124, 255, 255])
        self.grasp_planner = RandomPlanner([[3.14, 3.14], [0, 0], [0, 0]])
        self.motion_planner = ''

    def task_display(self):
        # results path
        self.time_monitor.dir = self.data_path
        self.time_monitor.csv_name = self.experiment_name+"_CostTime.csv"
        self.data_monitor.dir = self.data_path
        self.data_monitor.csv_name = self.experiment_name+"_TicTacToeData.csv"

        # sub-task display
        self.arm.go_home()
        for i in range(5):
            self.args["subtask_counter"] = i
            self.subtask_display()
            raw_input("waiting...")

    def subtask_display(self):
        # subtask data path
        subtask_name = "subtask_" + str(self.args["subtask_counter"])
        path = self.data_path + subtask_name + "/"
        self.image_monitor.dir = path

        # color image pre-process
        frame = self.camera.get_frame()
        color_image = frame.color_image[0]
        depth_image = frame.depth_image[0]
        sub_image = color_image[self.args["WorkSpace"][0][0]:self.args["WorkSpace"][0][1], 
                                self.args["WorkSpace"][1][0]:self.args["WorkSpace"][1][1],
                                :]
        cidata = {"Image": color_image}
        didata = {"Image": depth_image}
        self.image_monitor.img_name = subtask_name + "_rgb.jpg"
        self.publisher.sendData(cidata)
        self.image_monitor.img_name = subtask_name + "_d.jpg"
        self.publisher.sendData(didata)

        # segmentation
        start = time.time()
        bounding_box, mask, centers = self.localization_operator.display(sub_image)
        end = time.time()

        # data process
        tdata = {"Time": [subtask_name+' segmentation_time', end-start]}
        self.publisher.sendData(tdata)

        # recognition
        start = time.time()
        labels, _ = self.recognition_operator.display(centers, sub_image)
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        for center in centers:
            center[0] = self.args["WorkSpace"][1][0] + center[0]
            center[1] = self.args["WorkSpace"][0][0] + center[1]
        print(centers, labels)
        start = time.time()
        pose = self.grasp_planner.display(centers, labels)
        end = time.time()
        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)
        print(pose)
        # cv2.imshow("color", color_image)
        # cv2.waitKey(0)

        if pose is not None:
            xyz, avoidz = self.arm.uvd2xyz(pose[0], pose[1], depth_image, self.camera.get_intrinsics())
            pose[0], pose[1], pose[2] = xyz[0], xyz[1], 0.25
        # motion planning
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' motion_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # execution
        if pose is not None:
            start = time.time()
            self.arm.move_p(pose)
            pose[2] = 0.165
            self.arm.move_p(pose)
            self.gripper.close_gripper()
            end = time.time()
        else:
            start = time.time()
            end = time.time()

        tdata = {"Time": [subtask_name + ' execution_time', end - start]}
        self.publisher.sendData(tdata)

        tttdata = {"TicTacToeData": [bounding_box, centers, labels, pose]}
        self.publisher.sendData(tttdata)

        self.arm.go_home()
        self.gripper.open_gripper()
        
