import os
import sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

from Examples.Task import Task

class CalibrationTest(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(CalibrationTest, self).__init__(perception_system, manipulation_system, is_debug)

    def task_display(self):
        # robot system
        robot_arm = self.manipulation_system['Arm']
        robot_gripper = self.manipulation_system['End-effector']

        # sensors
        camera = self.perception_system['Camera']

        # test case
        robot_arm.calibrating()

    def subtask_display(self):
        pass



