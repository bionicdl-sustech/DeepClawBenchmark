import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from modules.calibration.Calibration2D import Calibration2D
from driver.arms.UR5Controller import UR5Controller


class TestCase(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(TestCase, self).__init__(perception_system, manipulation_system, is_debug)

    def task_display(self):
        # robot system
        robot_arm = self.manipulation_system['Arm']
        robot_gripper = self.manipulation_system['End-effector']

        # sensors
        camera = self.perception_system['camera']

        # test case
        print('testing perception system ...')
        try:
            color_image, _ = camera.getImage()
            print('successful')
        except:
            print('Test case failed! Error occupied in perception system!')

        print('testing manipulation system ...')
        try:
            robot_arm.goHome()
            robot_gripper.closeGripper()
            robot_gripper.openGripper()
            print('successful')
        except:
            print('Test case failed! Error occupied in manipulation system!')

        print('All test case have been done!')

    def subtask_display(self):
        pass
