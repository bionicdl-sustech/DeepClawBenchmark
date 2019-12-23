import os
import shutil
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("robot", type=str, choices=['ur10e', 'ur5', 'franka'], help="name of robot arm")
parser.add_argument("gripper", type=str, choices=['hande','suction_cup'], help="name of robot gripper")
parser.add_argument("sensor", type=str, choices=['realsense', 'kinect-azure'], help="name of sensor")
parser.add_argument("task", type=str, choices=['test', 'io-test', 'calibration', 'tic-tac-toe','Jigsaw','RandomClawMachine','CNNClawMachine_fc','CNNClawMachine','cv2ClawMachine','ClearTray','ClearTray_fc'], help="task name")
parser.add_argument("save", type=str, choices=['true', 'false'], help="whether saving program")
args = parser.parse_args()

ROBOT_NAME = args.robot
GRIPPER_NAME = args.gripper
SENSOR_NAME = args.sensor
TASK_NAME = args.task
FILES = []


def read_task_file(file_name, imp_lines):
    file = open(file_name)
    # ignored files
    #file_ignore = ["scipy","numpy","cv2","os","sys","pyrealsense2","yaml","socket","struct","math","time","optoforce","serial","copy"]
    file_save = ["config","data","driver","example","input_output","modules"]
    for line in file.readlines():
        strs = line.strip("\n").split(" ")
        if strs[0] == "import" or strs[0] == "from":
            tem = strs[1].split(".")
	    if(tem[0] not in file_save):
		continue
            if len(tem[0]) > 1:
                imp_line = "."
                for s in tem:
                    imp_line += "/"+s
                imp_lines.append(imp_line+".py")
                read_task_file(imp_line+".py", imp_lines)


def save_files(path, files):
    for file in files:
        name_list = file.split("/")[1:]
        target_path = path+"/".join(name_list[:-1])
        if not os.path.exists(target_path):
            os.makedirs(target_path)
        shutil.copy(file, path+"/".join(name_list))


def initial_sensors(sensor_name):
    if sensor_name == "realsense":
        from driver.sensors.camera.RealsenseController import RealsenseController
        FILES.append("./driver/sensors/camera/RealsenseController.py")
        read_task_file("./driver/sensors/camera/RealsenseController.py", FILES)
        realsense = RealsenseController()
        return realsense
    elif sensor_name == "kinect-azure":
        from driver.sensors.camera.AKinectController import AKinectController
        FILES.append("./driver/sensors/camera/AKinectController.py")
        read_task_file("./driver/sensors/camera/AKinectController.py", FILES)
        kinect = AKinectController()
        return kinect
    else:
        print("Not support for this sensor.")
        return None


def initial_gripper(gripper_name):
    if gripper_name == "hande":
        from driver.grippers.handE_controller.gripper_controller import HandEController
        FILES.append("./driver/grippers/handE_controller/gripper_controller.py")
        read_task_file("./driver/grippers/handE_controller/gripper_controller.py", FILES)
        gripper = HandEController()
        gripper.active_gripper()
        return gripper
    else:
        print("not support for this gripper.")
        return None


def initial_robot(robot_name):
    if robot_name == "ur10e":
        from driver.arms.UR10eController import UR10eController
        FILES.append("./driver/arms/UR10eController.py")
        read_task_file("./driver/arms/UR10eController.py", FILES)
        robot = UR10eController("/config/arms/ur10e.yaml")
        return robot
    elif robot_name == "ur5":
        from driver.arms.UR5Controller import UR5Controller
        FILES.append("./driver/arms/UR5Controller.py")
        read_task_file("./driver/arms/UR5Controller.py", FILES)
        robot = UR5Controller("/config/arms/ur5.yaml")
        return robot
    elif robot_name == "franka":
        from driver.arms.Franka.Franka import franka
        FILES.append("./driver/arms/Franka/Franka.py")
        read_task_file("./driver/arms/Franka/Franka.py", FILES)
        robot = franka("/config/arms/franka.yaml")
        return robot
    else:
        print("Don't support this robot!")
        return None


def initial_task(task_name, perception_system, manipulation_system, is_debug=False):
    if task_name == "test":
        from examples.TestCase import TestCase
        FILES.append("./examples/TestCase.py")
        read_task_file("./examples/TestCase.py", FILES)
        test_case = TestCase(perception_system, manipulation_system, is_debug)
        return test_case
    elif task_name == "io-test":
        from examples.IOTest import IOTest
        FILES.append("./examples/IOTest.py")
        read_task_file("./examples/IOTest.py", FILES)
        iotest = IOTest(perception_system, manipulation_system, is_debug)
        return iotest
    elif task_name == "calibration":
        from examples.Calibration import Calibration
        FILES.append("./examples/Calibration.py")
        read_task_file("./examples/Calibration.py", FILES)
        task = Calibration(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == "tic-tac-toe":
        from examples.TicTacToe3 import TicTacToe3
        FILES.append("./examples/TicTacToe3.py")
        read_task_file("./examples/TicTacToe3.py", FILES)
        task = TicTacToe3(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == "Jigsaw":
        from examples.Jigsaw_pickandplace import Jigsaw
        FILES.append("./examples/Jigsaw_pickandplace.py")
        read_task_file("./examples/Jigsaw_pickandplace.py", FILES)
        task = Jigsaw(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == "RandomClawMachine":
        from examples.RandomClawMachine import RandomClawMachine
        task = RandomClawMachine(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == "CNNClawMachine_fc":
        from examples.CNNClawMachine import CNNClawMachine_fc
        task = CNNClawMachine(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == "CNNClawMachine":
        from examples.CNNClawMachine import CNNClawMachine
        task = CNNClawMachine(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == 'cv2ClawMachine':
        from examples.cv2ClawMachine import cv2ClawMachine
        task = cv2ClawMachine(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == 'ClearTray':
        from examples.ClearTray import ClearTray
        task = ClearTray(perception_system, manipulation_system, is_debug)
        return task
    elif task_name == 'ClearTray_fc':
        from examples.ClearTray_fc import ClearTray_fc
        task = ClearTray_fc(perception_system, manipulation_system, is_debug)
        return task
    else:
        print('No such task.')
        return None


if __name__ == '__main__':
    sensor = initial_sensors(SENSOR_NAME)
    robot = initial_robot(ROBOT_NAME)
    if GRIPPER_NAME == "suction_cup":
		gripper = robot
    else:
        gripper = initial_gripper(GRIPPER_NAME)

    if robot is not None and sensor is not None:
        perception_system = {'Camera': sensor, 'Recorder': sensor}
        manipulation_system = {'Arm': robot, 'Gripper': gripper}
        if TASK_NAME != "calibration":
            robot.load_calibration_matrix()

        task = initial_task(TASK_NAME, perception_system, manipulation_system, is_debug=True)
        if task is not None:
            data_path = task.task_display()
            FILES.sort()
            if data_path is not None:
	            save_files(data_path+"code/", FILES)
