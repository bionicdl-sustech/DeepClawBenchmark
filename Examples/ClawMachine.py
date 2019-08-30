import os
import sys
import numpy as np
from PIL import Image

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from Examples.Task import Task
from Functions.ClawMachine.predictor import Predictor
from Functions.ClawMachine.Functions import *
from Functions.ClawMachine.success_label import *
from ToolKit.DataCollector import ImagePublisher, TimePublisher, Monitor

image_publisher = ImagePublisher('image_pub')
time_publisher = TimePublisher('time_pub')
data_collector = Monitor('ClawMachineTask4')
image_publisher.registerObserver(data_collector)
time_publisher.registerObserver(data_collector)

class ClawMachineTask(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(ClawMachineTask, self).__init__(perception_system, manipulation_system, is_debug)
        self.args = ''

    def subtask_display(self):
        # args initial
        camera = self.perception_system[0]
        robot_arm,  robot_gripper = self.manipulation_system[0], self.manipulation_system[1]
        Network, NUM_BOXES, WIDTH, crop_box, WorkSpace, ite = self.args[0], self.args[1], self.args[2], \
                                                              self.args[3], self.args[4], self.args[5]

        robot_arm.goHome()
        color_image, _ = camera.getImage()
        croped_image = Image.fromarray(color_image[crop_box[0]:crop_box[2], crop_box[1]:crop_box[3]])

        if self.is_debug:
            image_publisher.setData(color_image)
            image_publisher.setData(np.array(croped_image))

        robot_gripper.openGripper()

        # end to end network
        start_time = time.time()
        candidates_theta, candidates_probability, patches, boxes = end2end_display(image_publisher, camera, [Network, croped_image, NUM_BOXES, WIDTH], self.is_debug)
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'end2end', end_time - start_time])

        # multiple points motion planning step
        start_time = time.time()
        pick_location, place_location, best_theta = multiple_points_motion_planning(image_publisher, [robot_arm, robot_gripper],
                                                                                    [candidates_theta, candidates_probability,
                                                                                     croped_image, boxes,
                                                                                     NUM_BOXES, WIDTH, crop_box], self.is_debug)
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'multiple_points_motion_planning', end_time - start_time])

        # execution step
        start_time = time.time()
        execution_display(image_publisher, [robot_arm, robot_gripper], [pick_location, place_location, best_theta])
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'execution', end_time - start_time])

    def task_display(self):
        # parameters initial
        NUM_BOXES = 20
        WIDTH = 120
        Network = Predictor(root_path+'/Functions/ClawMachine/checkpoint')
        crop_box = [52, 331, 400, 1135]
        WorkSpace = [-0.21, -0.37, 0.21, -0.68]

        for i in range(18):
            self.args = [Network, NUM_BOXES, WIDTH, crop_box, WorkSpace, i]
            self.subtask_display()
            grasp_label = raw_input('grasp label[0/1/2]:')
            if self.is_debug:
                time_publisher.setData([str(i)+'grasp label', str(grasp_label)])
