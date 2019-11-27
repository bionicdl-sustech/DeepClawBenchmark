import cv2
import numpy as np


class Display(object):
    def __init__(self):
        self.flag = 0
        self.input_data = np.zeros((1280, 720, 3))
        self.output_data = np.zeros((1280, 720, 3))
        self.step = ["Step", "Segmentation", "Recognition", "Motion Planning", "Execution"]
        self.result = ["Result", "--", "--", "--", "--"]
        self.SOTA = ["SOTA", "--", "--", "--", "--"]
        self.ground_truth = ["Ground Truth", "--", "--", "--", "--"]
        self.time_cost = ["Time Cost", "--", "--", "--", "--"]

    def start(self):
        self.display()
        # main loop here
        while self.flag == 0:
            print("loop")

    def display(self):
        # print all information
        pass

    def data_update(self, line, input_data, output_data, result, SOTA, ground_truth, time_cost):
        self.input_data = input_data
        self.output_data = output_data
        self.result[line] = result
        self.SOTA[line] = SOTA
        self.ground_truth[line] = ground_truth
        self.time_cost[line] = time_cost

    def stop(self):
        self.flag = 1
