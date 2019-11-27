import numpy as np
import cv2
import sys
import os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)


class Monitor(object):
    def __init__(self, camera):
        self.flag = 0
        self.camera = camera
        self.real_time_data = None
        self.localization_data = None
        self.recognition_data = None
        self.grasp_planning_data = None
        self.motion_planning_data = None
        self.step = ["Step", "Result", "SOTA", "Ground Truth", "Time Cost"]
        self.localization_metrics = {"Step": "Localization", "Result": "--",
                                     "SOTA": "--", "Ground Truth": "--", "Time Cost": "--"}
        self.recognition_metrics = {"Step": "Recognition", "Result": "--",
                                     "SOTA": "--", "Ground Truth": "--", "Time Cost": "--"}
        self.grasp_planning_metrics = {"Step": "Grasp Planning", "Result": "--",
                                     "SOTA": "--", "Ground Truth": "--", "Time Cost": "--"}
        self.motion_planning_metrics = {"Step": "Motion Planning", "Result": "--",
                                     "SOTA": "--", "Ground Truth": "--", "Time Cost": "--"}

    def start(self):
        # main loop here
        self.data_initial()
        while self.flag == 0:
            self.display()

    def display(self):
        # print all information
        left_image = cv2.hconcat(cv2.vconcat(self.localization_data, self.grasp_planning_data),
                                 cv2.vconcat(self.recognition_data, self.motion_planning_data))
        right_image = self.real_time_data
        top = cv2.hconcat(right_image, left_image)
        bottom = np.zeros((top.shape[0], 300, top.shape[2]))
        image = cv2.vconcat(top, bottom)
        cv2.imshow("Display", image)
        cv2.waitKey(1)

    def data_update(self):
        pass

    def data_initial(self):
        self.real_time_data = self.camera.getFrame().color_image
        self.localization_data = self.get_subimage(np.zeros(self.real_time_data.shape))
        self.recognition_data = self.get_subimage(np.zeros(self.real_time_data.shape))
        self.grasp_planning_data = self.get_subimage(np.zeros(self.real_time_data.shape))
        self.motion_planning_data = self.get_subimage(np.zeros(self.real_time_data.shape))
        # self.localization_data = cv2.imread(_root_path+"/data/monitor/localization_data.jpg")
        # self.recognition_data = cv2.imread(_root_path+"/data/monitor/recognition_data.jpg")
        # self.grasp_planning_data = cv2.imread(_root_path+"/data/monitor/grasp_planning_data.jpg")
        # self.motion_planning_data = cv2.imread(_root_path+"/data/monitor/motion_planning_data.jpg")

    def get_subimage(self, image):
        shape = image.shape
        subimage = cv2.resize(image, (int(shape[0]/2.0), int(shape[1]/2.0)), interpolation=cv2.INTER_LINEAR)
        return subimage


    def stop(self):
        self.flag = 1
