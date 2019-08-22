import cv2, time
import numpy as np
from math import pi
from Driver.Camera.RealsenseController import RealsenseController
from ToolKit.Calibration2D import Calibration2D
import Examples.TicTacToe as tic_tac_toe

def ur10_display():
    from Driver.UR10e.UrController import URController
    from Driver.UR10e.handE_controller.gripper_controller import HandEController
    realsense = RealsenseController()
    ur = URController()
    end_effector = HandEController()
    c2d = Calibration2D()

    ur.calibration_tool = c2d
    ur.calibration_tool.xy_set = [[0.43976,0.72971], [0.45804, 0.49703],
                                 [0.23657, 0.74348], [0.260350, 0.500420]]
    ur.calibration_tool.uv_set = [[896,434], [678,456],
                                 [904,244], [676,274]]
    ur.calibration_tool.matrix_update()

    end_effector.openGripper()
    tic_tac_toe.auto_display(ur, end_effector, realsense)

def franka_display():
    from Driver.FrankaPanda.FrankaController import FrankaController
    from Driver.FrankaPanda.FrankaEndeffectorController import FrankaEndeffectorController
    realsense = RealsenseController()
    franka = FrankaController()
    end_effector = FrankaEndeffectorController()
    c2d = Calibration2D()

    franka.calibration_tool = c2d
    franka.calibration_tool.xy_set = [[0.232997579286, 0.278699656702], [0.622929389307, 0.284811723732],
                                 [0.630048128924, -0.200942368032], [0.242755221618, -0.20808202166]]
    franka.calibration_tool.uv_set = [[297, 98], [671, 103],
                                 [668, 566], [296, 570]]
    franka.calibration_tool.matrix_update()

    end_effector.openGripper()
    tic_tac_toe.auto_display(franka, end_effector, realsense)

def realsense_test():
    realsense = RealsenseController()
    i=0
    while i<=5:
        c, d, l, r = realsense.getImage()
        # _, cc, uvr = tic_tac_toe.pieces_detect(c, 'O')
        # cc = cc[100:400, 400:800]
        cv2.imshow('c', c)
        cv2.waitKey(0)
        i+=1

def aubo_display():
    from Driver.Aubo.AuboController import AuboController
    realsense = RealsenseController()
    aubo = AuboController()
    c2d = Calibration2D()

    aubo.calibration_tool = c2d
    # aubo.calibration_tool.xy_set = [[0.2796, -0.2204],[0.3515, -0.0822],[0.4919, -0.0112],[0.6336, -0.1532]]
    # aubo.calibration_tool.uv_set = [[488, 204], [624, 158], [759, 199], [763, 378]]
    aubo.calibration_tool.xy_set = [[0.329053, -0.133206], [0.378447, -0.038181], [0.478084, -0.017679], [0.553431, -0.148724]] #[0.407044, -0.25476]
    aubo.calibration_tool.uv_set = [[550, 220], [640, 186], [719, 232], [690, 364]]#[524, 345]
    aubo.calibration_tool.matrix_update()

    tic_tac_toe.auto_display(aubo, aubo, realsense)
    # tic_tac_toe.display(aubo, aubo, realsense)

def denso_display():
    from Driver.Cobotta.CobottaController import CobottaController
    realsense = RealsenseController()
    cobotta = CobottaController()
    c2d = Calibration2D()
    cobotta.calibrating(xy_set=[[0.13, 0.1], [0.13, -0.15],
                                [0.3, 0.1], [0.3, -0.15]],
                        uv_set=[[596, 332], [619, 823],
                                [265, 339], [280, 835]],
                        c2d=c2d)
    perception_system = {'Camera': realsense}
    maniuplation_system = {'Arm': cobotta, 'End-effector': cobotta}
    tic_tac_toe.task_display(perception_system, maniuplation_system, is_debug=True)

if __name__ == '__main__':
    # franka_display()
    # ur10_display()
    # aubo_display()
    # realsense_test()
    denso_display()
