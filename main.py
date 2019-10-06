import time
import argparse
import cv2
from Driver.Camera.RealsenseController import RealsenseController

parser = argparse.ArgumentParser()
parser.add_argument("robot", type=str, choices=['denso', 'ur10e'], help="name of robot arm")
parser.add_argument("task", type=str, choices=['test', 'calibration-test', 'collect-data',
                                               'tic-tac-toe', 'claw-machine', 'jigsaw-puzzle'], help="task name")
args = parser.parse_args()

robot_name = args.robot
task_name = args.task

def ur10_test():
    from Driver.UR10e.UrController import URController
    realsense = RealsenseController(serial_id='825312073784')
    ur = URController()
    ur.goHome()
    # ur.calibrating(realsense)
    ur.matrix_load()
    print(ur._R, ur._t)
    ci, info = realsense.getImage()
    cv2.imshow('current', ci)
    cv2.waitKey()
    u = int(raw_input('u: '))
    v = int(raw_input('v: '))
    xyz, _ = ur.uvd2xyz(u, v, info[0], realsense.get_depth_scale())
    x, y, cz = xyz[0], xyz[1], xyz[2]
    print(x, y, cz, _)
    raw_input('waiting...')
    z = 0.4
    ur.move([[x, y, z], [3.14, 0, 0]])

    # c2d = Calibration2D()
    #
    # ur.calibration_tool = c2d
    # ur.calibration_tool.xy_set = [[0.3429, -0.0830], [0.7317, -0.0842],
    #                               [0.7314, -0.5725], [0.3441, -0.5687]]
    # ur.calibration_tool.uv_set = [[493, 69], [876, 65],
    #                               [867, 533], [506, 536]]
    # ur.calibration_tool.matrix_update()
    #
    # xy = ur.calibration_tool.cvt(698, 298)
    #
    # ur.movej(xy[0], xy[1], 0.18, 3.14, 0, 0)

def ur10e_calibration():
    realsense = RealsenseController()
    from Driver.UR10e.UrController import URController
    from ToolKit.Calibration3D import image_callback
    ur10e = URController()
    ix, iy, iz = 0.5, -0.25, 0.35
    ur10e.movej(ix, iy, iz, 0, 0, 3.14)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                ur10e.movej(ix+0.02*i, iy-0.02*j, iz+0.02*k, 0, 0, 3.14)
                c, o = realsense.getImage()
                _, bc = image_callback(c, o[0], realsense.depth_scale)
                print(_)
                cv2.imshow('bc', bc)
                cv2.waitKey(0)

def realsense_test():
    # realsense = RealsenseController(serial_id='825312073784')
    realsense = RealsenseController(serial_id='821312062518')
    # print(realsense.get_device())
    i=0
    while i<=5:
        c, o = realsense.getImage()
        cv2.imshow('c', c)
        cv2.waitKey(0)
        i+=1

def multiple_threads_test():
    import thread
    from ToolKit.VideoRecorder import VideoRecorder
    realsense = RealsenseController(serial_id='821312062518')
    recorder = VideoRecorder(camera=realsense)
    thread.start_new_thread(recorder.start, ())

    raw_input("recording...")
    recorder.stop()


def initial_robot(robot_name):
    if robot_name=='denso':
        import os
        os.system(
            "gnome-terminal -e 'bash -c \"sh ./Driver/Cobotta/initial.sh;exec bash\"'")
        time.sleep(5)
        os.system(
            "gnome-terminal -e 'bash -c \"sh ./Driver/Cobotta/server.sh;exec bash\"'")
        time.sleep(5)
        from Driver.Cobotta.CobottaController import CobottaController
        robot = CobottaController()
        return robot
    if robot_name == 'ur10e':
        from Driver.UR10e.UrController import URController
        robot = URController()
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
        from Examples.CalibrationTest import CalibrationTest
        test_case = CalibrationTest(perception_system, manipulation_system, is_debug)
        return test_case
    elif task_name=='collect-data':
        from Examples.CollectData import CollectData
        collect_data = CollectData(perception_system, manipulation_system, is_debug)
        return collect_data
    elif task_name=='tic-tac-toe':
        from Examples.TicTacToe import TicTacToeTask
        tic_tac_toe = TicTacToeTask(perception_system, manipulation_system, is_debug)
        return tic_tac_toe
    elif task_name=='claw-machine':
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
    # ur10_test()
    # multiple_threads_test()
    # realsense_test()
    realsense1 = RealsenseController(serial_id='825312073784')
    realsense2 = RealsenseController(serial_id='821312062518')
    robot = initial_robot(robot_name)
    robot.matrix_load()

    if robot!=None:
        perception_system = {'Camera': realsense1, 'Recorder': realsense2}
        maniuplation_system = {'Arm': robot, 'End-effector': robot}

        task = initial_task(task_name, perception_system, maniuplation_system, is_debug=True)
        if task!=None:
            task.task_display()
