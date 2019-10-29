import time
import argparse
import cv2
from driver.sensors.camera.RealsenseController import RealsenseController

parser = argparse.ArgumentParser()
parser.add_argument("robot", type=str, choices=['denso', 'ur10e'], help="name of robot arm")
parser.add_argument("task", type=str, choices=['test', 'calibration-test', 'collect-data', 'trash-classification',
                                               'soft-gripper-grasp',
                                               'tic-tac-toe', 'claw-machine', 'jigsaw-puzzle'], help="task name")
args = parser.parse_args()

ROBOT_NAME = args.robot
TASK_NAME = args.task


def initial_robot(robot_name):
    if robot_name == "ur10e":
        from driver.arms.UR10eController import UR10eController
        robot = UR10eController()
        return robot
    elif robot_name == "ur5":
        from driver.arms.UR5Controller import UR5Controller
        robot = UR5Controller()
        return robot
    else:
        print("Don't support this robot!")
        return None


def initial_task(task_name, perception_system, manipulation_system, is_debug=False):
    if task_name == "test":
        from examples.TestCase import TestCase
        test_case = TestCase(perception_system, manipulation_system, is_debug)
        return test_case
    else:
        print('No such task.')
        return None


if __name__ == '__main__':
    realsense1 = RealsenseController(serial_id='825312073784')
    realsense2 = RealsenseController(serial_id='821312062518')
    robot = initial_robot(ROBOT_NAME)
    robot.matrix_load()

    if robot is not None:
        perception_system = {'camera': realsense1, 'Recorder': realsense2}
        manipulation_system = {'Arm': robot, 'End-effector': robot}

        task = initial_task(TASK_NAME, perception_system, manipulation_system, is_debug=True)
        if task is not None:
            task.task_display()
