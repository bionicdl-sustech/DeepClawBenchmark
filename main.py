import cv2
import argparse
from Driver.Camera.RealsenseController import RealsenseController
from ToolKit.Calibration2D import Calibration2D

parser = argparse.ArgumentParser()
parser.add_argument("robot", type=str, choices=['denso', 'ur'], help="name of robot arm")
parser.add_argument("task", type=str, choices=['test', 'calibration-test',
                                               'tic-tac-toe', 'claw-machine', 'jigsaw-puzzle'], help="task name")
args = parser.parse_args()

robot_name = args.robot
task_name = args.task

# def ur5_display():
#     from Driver.UR5.UR5Controller import UR5Controller
#     realsense = RealsenseController()
#     ur5 = UR5Controller()
#     c2d = Calibration2D()
#
#     ur5.calibration_tool = c2d
#     ur5.calibration_tool.xy_set = [[-0.25318, -0.32069], [-0.26039, -0.70754],
#                                    [0.23002, -0.71088], [0.23564, -0.32275]]
#     ur5.calibration_tool.uv_set = [[437, 254], [435, 599],
#                                    [875, 601], [874, 253]]
#     ur5.calibration_tool.matrix_update()
#
# def ur10_display():
#     from Driver.UR10e.UrController import URController
#     from Driver.UR10e.handE_controller.gripper_controller import HandEController
#     realsense = RealsenseController()
#     ur = URController()
#     end_effector = HandEController()
#     c2d = Calibration2D()
#
#     ur.calibration_tool = c2d
#     ur.calibration_tool.xy_set = [[0.43976, 0.72971], [0.45804, 0.49703],
#                                   [0.23657, 0.74348], [0.260350, 0.500420]]
#     ur.calibration_tool.uv_set = [[896, 434], [678, 456],
#                                   [904, 244], [676, 274]]
#     ur.calibration_tool.matrix_update()
#
#     end_effector.openGripper()
#     tic_tac_toe.auto_display(ur, end_effector, realsense)
#
# def franka_display():
#     from Driver.FrankaPanda.FrankaController import FrankaController
#     from Driver.FrankaPanda.FrankaEndeffectorController import FrankaEndeffectorController
#     realsense = RealsenseController()
#     franka = FrankaController()
#     end_effector = FrankaEndeffectorController()
#     c2d = Calibration2D()
#
#     franka.calibration_tool = c2d
#     franka.calibration_tool.xy_set = [[0.232997579286, 0.278699656702], [0.622929389307, 0.284811723732],
#                                       [0.630048128924, -0.200942368032], [0.242755221618, -0.20808202166]]
#     franka.calibration_tool.uv_set = [[297, 98], [671, 103],
#                                       [668, 566], [296, 570]]
#     franka.calibration_tool.matrix_update()
#
#     end_effector.openGripper()
#     tic_tac_toe.auto_display(franka, end_effector, realsense)
#
# def realsense_test():
#     realsense = RealsenseController()
#     i=0
#     while i<=5:
#         c, _ = realsense.getImage()
#         # _, cc, uvr = tic_tac_toe.pieces_detect(c, 'O')
#         # cc = cc[100:400, 400:800]
#         cv2.imshow('c', c)
#         cv2.waitKey(0)
#         i+=1
#
# def aubo_display():
#     from Driver.Aubo.AuboController import AuboController
#     realsense = RealsenseController()
#     aubo = AuboController()
#     c2d = Calibration2D()
#
#     aubo.calibration_tool = c2d
#     aubo.calibration_tool.xy_set = [[0.329053, -0.133206], [0.378447, -0.038181],
#                                     [0.478084, -0.017679], [0.553431, -0.148724]]
#     aubo.calibration_tool.uv_set = [[550, 220], [640, 186],
#                                     [719, 232], [690, 364]]
#     aubo.calibration_tool.matrix_update()
#
#     tic_tac_toe.auto_display(aubo, aubo, realsense)
#
# # def circle_detect():
# #     import numpy as np
# #     from ClassicalAlgorithms.CVAlgorithm import CVAlgorithm
# #     CA = CVAlgorithm()
# #     realsense = RealsenseController()
# #     i = 0
# #     while i <= 5:
# #         color_image, _ = realsense.getImage()
# #         i += 1
# #     gray = CA.color2gray(color_image)
# #     # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=28,
# #     #                            maxRadius=33)
# #     circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
# #                                maxRadius=25)
# #     uvr = []
# #     if circles is not None:
# #         circles = np.uint16(np.around(circles))  # shape (1, n, 3)
# #         for i in range(circles.shape[1]):
# #             u, v, r = circles[0, i]
# #             uvr.append([u, v, r])
# #             cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
# #             print(u, v)
# #     cv2.imshow('c', color_image)
# #     cv2.waitKey(0)
#
# # def board_detect():
# #     import numpy as np
# #     camera = RealsenseController()
# #     centers = []
# #     from ClassicalAlgorithms.CVAlgorithm import CVAlgorithm
# #     CA = CVAlgorithm()
# #     while 1:
# #         color_background, _ = camera.getImage()
# #         contours = CA.find_contours(color_background, threshold=50)
# #         # cv2.drawContours(color_background, contours, -1, (0, 255, 0), 1)
# #         # cv2.imshow('background', color_background)
# #         # cv2.waitKey(1)
# #         contours = CA.contours_area_filter(contours, 5500, 6500)
# #         cv2.drawContours(color_background, contours, -1, (0, 255, 0), 1)
# #         cv2.imshow('background2', color_background)
# #         cv2.waitKey(1)
# #         centers = CA.find_contours_center(contours)
# #         # cv2.drawContours(c, contours, -1, (0, 255, 0), 1)
# #         cx, cy = 0, 0
# #         for center in centers:
# #             # cv2.circle(color_background, (center[0], center[1]), 1, (0, 255, 0), 2)
# #             cx+=center[0]
# #             cy+=center[1]
#
# def denso_display():
#     from Driver.Cobotta.CobottaController import CobottaController
#     realsense = RealsenseController()
#     cobotta = CobottaController()
#     c2d = Calibration2D()
#     cobotta.calibrating(xy_set=[[0.1634, 0.16309], [0.29218, 0.1588],
#                                 [0.15783, -0.16825], [0.28415, -0.17014]],
#                         uv_set=[[1121, 52], [1135, 375],
#                                 [331, 97], [338, 400]],
#                         c2d=c2d)
#     perception_system = {'Camera': realsense}
#     maniuplation_system = {'Arm': cobotta, 'End-effector': cobotta}
#     tic_tac_toe.task_display(perception_system, maniuplation_system, is_debug=True)
#     # clawmachine.task_display(perception_system, maniuplation_system, is_debug=True)

def initial_robot(robot_name):
    if robot_name=='denso':
        import os
        os.system("gnome-terminal -e 'bash -c \"sh ./Driver/Cobotta/initial.sh;exec bash\"'")
        os.system("gnome-terminal -e 'bash -c \"sh ./Driver/Cobotta/server.sh;exec bash\"'")
        from Driver.Cobotta.CobottaController import CobottaController
        robot = CobottaController()
        robot.calibrating()
        return robot
    else:
        print("Don't support this robot!")
        return None

def initial_task(task_name, perception_system, manipulation_system, is_debug=False):
    if task_name=='test':
        from Examples.TestCase import TestCase
        test_case = TestCase(perception_system, manipulation_system, is_debug)
        return test_case
    elif task_name=='calibration-test':
        print('Waiting for update ...')
        return None
    elif task_name=='tic-tac-toe':
        from Examples.TicTacToe import TicTacToeTask
        tic_tac_toe = TicTacToeTask(perception_system,manipulation_system, is_debug)
        return tic_tac_toe
    elif task_nam=='claw-machine':
        from Examples.ClawMachine import ClawMachineTask
        claw_machine = ClawMachineTask(perception_system, manipulation_system, is_debug)
        return claw_machine
    elif task_name=='jigsaw_puzzle':
        print('Waiting for update ...')
        return None
    else:
        print('Has no such task.')
        return None

if __name__ == '__main__':
    realsense = RealsenseController()
    robot = initial_robot(robot_name)

    perception_system = {'Camera': realsense}
    maniuplation_system = {'Arm': robot, 'End-effector': robot}

    task = initial_task(task_name, perception_system, maniuplation_system, is_debug=True)
    task.task_display()
