import random
import time
import sys
import os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

import ToolKit.success_label as label
from Examples.Task import Task
from ToolKit.DataCollector import ImagePublisher, GraspingDataPublisher, Monitor

image_publisher = ImagePublisher('image_pub')
graspdata_publisher = GraspingDataPublisher('grasp_data_pub')

data_collector = Monitor('TrainingData')
image_publisher.registerObserver(data_collector)
graspdata_publisher.registerObserver(data_collector)

class CollectData(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(CollectData, self).__init__(perception_system, manipulation_system, is_debug)
        self.args = ''

    def subtask_display(self):
        # args initial
        camera = self.perception_system['Camera']
        robot_arm,  robot_gripper = self.manipulation_system['Arm'], self.manipulation_system['End-effector']
        place_z = 0.2
        pick_z = 0.18

        color_image, _ = camera.getImage()
        image_publisher.setData(color_image)

        # x = random.uniform(0.103, 0.594)
        # y = random.uniform(0.734, 0.346)
        x = random.uniform(0.370, 0.715)
        y = random.uniform(-0.542, -0.092)
        angel = random.uniform(-1.57, 1.57)

        goal_pose = [[x, y, place_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        goal_pose = [[x, y, pick_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        robot_gripper.closeGripper()
        goal_pose = [[x, y, place_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        robot_arm.goHome()
        time.sleep(1)

        color_image2, _ = camera.getImage()
        success_label, imagegray = label.success_label(color_image, color_image2)
        graspdata_publisher.setData([x, y], angel, success_label)

        x = random.uniform(0.4, 0.65)
        y = random.uniform(-0.45, -0.15)
        angel = random.uniform(-1.57, 1.57)
        print(x, y, angel)

        goal_pose = [[x, y, place_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        # robot_arm.verifyPostion([x, y, place_z, 3.14, 0, angel])
        robot_gripper.openGripper()
        robot_arm.goHome()
        time.sleep(1)

    def task_display(self):
        # parameters initial
        self.args = []
        robot_arm, robot_gripper = self.manipulation_system['Arm'], self.manipulation_system['End-effector']
        robot_gripper.openGripper()
        robot_arm.goHome()

        for i in range(3):
            self.subtask_display()
