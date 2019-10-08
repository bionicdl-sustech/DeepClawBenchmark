import socket
import thread
import time
import sys
import os
import gc

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from ToolKit.VideoRecorder import VideoRecorder
import ToolKit.success_label as label
from Examples.Task import Task
from ToolKit.DataCollector import ImagePublisher, GraspingDataPublisher, Monitor
from Functions.TrashSorting.TrashClassifier import *

SAVE_NAME = 'trash_sorting_0'

image_publisher = ImagePublisher('image_pub')
# graspdata_publisher = GraspingDataPublisher('grasp_data_pub')

data_collector = Monitor(SAVE_NAME)
image_publisher.registerObserver(data_collector)
# graspdata_publisher.registerObserver(data_collector)

class TrashClassificatiom(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(TrashClassificatiom, self).__init__(perception_system, manipulation_system, is_debug)
        self.args = ''

    def pick(self, robot, pick_pose, pick_position):
        robot_arm = robot[0]
        robot_gripper = robot[1]
        pose1 = pick_pose[0]
        pose2 = pick_pose[1]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((robot_arm._robot_ip, robot_arm._port))
        command = "def process():\n"
        command += "movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (pose1[0], pose1[1], pose1[2],
                                                                            pose1[3], pose1[4], pose1[5],
                                                                            0.5, 0.6)
        command += "set_digital_out(4,True)\n"
        command += "movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (pose2[0], pose2[1], pose2[2],
                                                                            pose2[3], pose2[4], pose2[5],
                                                                            0.5, 0.6)
        command += "end\n"
        s.send(str.encode(command))
        robot_arm.verifyPosition(pick_position)
        robot_gripper.closeGripper()
        time.sleep(0.5)
        s.close()

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((robot_arm._robot_ip, robot_arm._port))

        back_command = "def process():\n"
        back_command += "movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (pose1[0], pose1[1], pose1[2],
                                                                                 pose1[3], pose1[4], pose1[5],
                                                                                 0.5, 0.6)
        back_command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (-72.94 * 3.14159 / 180.0,
                                                                                -80.84 * 3.14159 / 180.0,
                                                                                -130.1 * 3.14159 / 180.0,
                                                                                -51.87 * 3.14159 / 180.0,
                                                                                111.72 * 3.14159 / 180.0,
                                                                                -161.44 * 3.14159 / 180.0,
                                                                                0.5, 0.6)
        back_command += "end\n"
        s.send(str.encode(back_command))
        robot_arm.verifyJoints([0.03, -0.54, 0.4, 3.14, -0.4, 0])
        s.close()

    def place(self, robot, place_pose, place_position):
        robot_arm = robot[0]
        robot_gripper = robot[1]
        pose1 = place_pose[0]
        pose2 = place_pose[1]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((robot_arm._robot_ip, robot_arm._port))
        command = "def process():\n"
        command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (6.69 * 3.14159 / 180.0,
                                                                           -78.35 * 3.14159 / 180.0,
                                                                           -120.47 * 3.14159 / 180.0,
                                                                           -71.18 * 3.14159 / 180.0,
                                                                           89 * 3.14159 / 180.0,
                                                                           -83.20 * 3.14159 / 180.0,
                                                                           0.5, 0.6)
        command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (pose1[0] * 3.14159 / 180.0,
                                                                           pose1[1] * 3.14159 / 180.0,
                                                                           pose1[2] * 3.14159 / 180.0,
                                                                           pose1[3] * 3.14159 / 180.0,
                                                                           pose1[4] * 3.14159 / 180.0,
                                                                           pose1[5] * 3.14159 / 180.0,
                                                                           0.5, 0.6)
        command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (pose2[0] * 3.14159 / 180.0,
                                                                           pose2[1] * 3.14159 / 180.0,
                                                                           pose2[2] * 3.14159 / 180.0,
                                                                           pose2[3] * 3.14159 / 180.0,
                                                                           pose2[4] * 3.14159 / 180.0,
                                                                           pose2[5] * 3.14159 / 180.0,
                                                                           0.5, 0.6)
        command += "end\n"
        s.send(str.encode(command))
        robot_arm.verifyJoints(place_position)
        robot_gripper.openGripper()
        time.sleep(0.5)
        robot_gripper.closeGripper()
        s.close()

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((robot_arm._robot_ip, robot_arm._port))
        back_command = "def process():\n"
        back_command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (pose1[0] * 3.14159 / 180.0,
                                                                                pose1[1] * 3.14159 / 180.0,
                                                                                pose1[2] * 3.14159 / 180.0,
                                                                                pose1[3] * 3.14159 / 180.0,
                                                                                pose1[4] * 3.14159 / 180.0,
                                                                                pose1[5] * 3.14159 / 180.0,
                                                                                0.5, 0.6)
        back_command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (6.69 * 3.14159 / 180.0,
                                                                                -78.35 * 3.14159 / 180.0,
                                                                                -120.47 * 3.14159 / 180.0,
                                                                                -71.18 * 3.14159 / 180.0,
                                                                                89 * 3.14159 / 180.0,
                                                                                -83.20 * 3.14159 / 180.0,
                                                                                0.5, 0.6)
        back_command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (-72.94 * 3.14159 / 180.0,
                                                                                -80.84 * 3.14159 / 180.0,
                                                                                -130.1 * 3.14159 / 180.0,
                                                                                -51.87 * 3.14159 / 180.0,
                                                                                111.72 * 3.14159 / 180.0,
                                                                                -161.44 * 3.14159 / 180.0,
                                                                                0.5, 0.6)
        back_command += "end\n"
        s.send(str.encode(back_command))
        robot_arm.verifyJoints([0.03, -0.54, 0.4, 3.14, -0.4, 0])
        s.close()


    def subtask_display(self):
        # args initial
        camera = self.perception_system['Camera']
        robot_arm,  robot_gripper = self.manipulation_system['Arm'], self.manipulation_system['End-effector']

        time.sleep(0.5)
        color_image, info = camera.getImage()

        image, uv = locate_trash(color_image)
        classification_label = predict(image)
        image_publisher.setData(image)

        xyz, avoid_z = robot_arm.uvd2xyz(uv[0], uv[1], info[0], camera.get_depth_scale())
        print(uv, classification_label)
        print(xyz, avoid_z)  #
        x, y, z = xyz[0], xyz[1], avoid_z + 0.195
        if z < 0.27:
            z2 = 0.27
        elif z > 0.65:
            z2 = 0.65
        else:
            z2 = z
        z1 = 0.54
        angle = 0
        print(z2)

        self.pick([robot_arm, robot_gripper], [[x, y, z1, 3.14, 0, angle], [x, y, z2, 3.14, 0, angle]],
                  [x, y, z2, 3.14, 0, angle])
        # robot_arm.move([[x, y, z1], [3.14, 0, angle]])
        # robot_gripper.openGripper()
        #
        # robot_arm.move([[x, y, z2], [3.14, 0, angle]])
        # robot_gripper.closeGripper()
        #
        # robot_arm.move([[x, y, z1], [3.14, 0, angle]])
        # robot_arm.goHome()

        # success label judgement
        time.sleep(0.5)
        color_image2, _ = camera.getImage()
        success_label, imagegray = label.success_label(color_image, color_image2)
        # graspdata_publisher.setData([u, v, x, y, pick_z], angle, success_label)

        if success_label == 1:
            # placing
            # joint0 = [6.69, -78.35, -120.47, -71.18, 89, -83.20]
            # pose0 = [0.55364, -0.10892, 0.48042, 3.14, 0, 0]
            # robot_arm.move([joint0, pose0], useJoint=True)
            if classification_label == 0:
                joint1 = [95.07, -74.49, -117.64, -77.88, 89.90, 5.20]
                # pose1 = [0.12640, 0.52815, 0.53499, 3.14, 0, 0]
                joint2 = [95.07, -76.60, -123.44, -69.96, 89.90, 5.20]
                pose2 = [0.12640, 0.52815, 0.46485, 3.14, 0, 0]
            elif classification_label == 1:
                joint1 = [68.81, -86.19, -107.96, -75.85, 89.87, -21.07]
                # pose1 = [0.38998, 0.52816, 0.53621, 3.14, 0, 0]
                joint2 = [68.81, -88.25, -113.11, -68.65, 89.87, -21.07]
                pose2 = [0.38998, 0.52816, 0.46894, 3.14, 0, 0]
                # self.place(robot_arm, robot_gripper, [joint1, pose1], [joint2, pose2])
            elif classification_label == 2:
                joint1 = [50.30, -104.80, -89.47, -75.72, 89.84, -39.62]
                # pose1 = [0.66336, 0.52815, 0.51593, 3.14, 0, 0]
                joint2 = [50.30, -106.26, -93.20, -70.54, 89.84, -39.63]
                pose2 = [0.66336, 0.52815, 0.46233, 3.14, 0, 0]
                # self.place(robot_arm, robot_gripper, [joint1, pose1], [joint2, pose2])
            elif classification_label == 3:
                joint1 = [38.24, -125.91, -57.18, -86.90, 89.81, -51.70]
                # pose1 = [0.93181, 0.51322, 0.52985, 3.14, 0, 0]
                joint2 = [38.24, -127.00, -63.83, -79.16, 89.81, -51.72]
                pose2 = [0.93181, 0.51322, 0.44634, 3.14, 0, 0]
                # self.place(robot_arm, robot_gripper, [joint1, pose1], [joint2, pose2])
            else:
                joint1 = []
                joint2 = []
                pose2 = []
            self.place([robot_arm, robot_gripper], [joint1, joint2], pose2)
            # robot_arm.goHome()

    def task_display(self):
        # parameters initial
        r_camera = self.perception_system['Recorder']
        video_folder = _root_path + '/Data/' + SAVE_NAME + '/video/'
        if not os.path.exists(video_folder):
            os.makedirs(video_folder)

        robot_arm, robot_gripper = self.manipulation_system['Arm'], self.manipulation_system['End-effector']
        robot_gripper.closeGripper()
        robot_arm.goHome()

        for i in range(50):
            recorder = VideoRecorder(camera=r_camera)
            recorder.video_dir = video_folder+str(i)+'.avi'
            thread.start_new_thread(recorder.start, ())

            print('Subtask: ', i)
            self.subtask_display()

            recorder.stop()
            del recorder
            gc.collect()
            time.sleep(1)
            raw_input('waiting...')
