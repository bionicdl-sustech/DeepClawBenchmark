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
from Functions.SoftGripper.Predictor import Predictor

SAVE_NAME = 'test_case_1'

image_publisher = ImagePublisher('image_pub')
graspdata_publisher = GraspingDataPublisher('grasp_data_pub')

data_collector = Monitor(SAVE_NAME)
image_publisher.registerObserver(data_collector)
graspdata_publisher.registerObserver(data_collector)

class SoftGripperGrasp(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(SoftGripperGrasp, self).__init__(perception_system, manipulation_system, is_debug)
        self.args = []

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

    def place(self, robot, place_position=None):
        if place_position is None:
            place_position = [-0.22923, -0.49763, 0.40082, 3.14, 0, 0]
        robot_arm = robot[0]
        robot_gripper = robot[1]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((robot_arm._robot_ip, robot_arm._port))
        command = "def process():\n"
        command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (-96.45 * 3.14159 / 180.0,
                                                                           -80.16 * 3.14159 / 180.0,
                                                                           -127.74 * 3.14159 / 180.0,
                                                                           -62.16 * 3.14159 / 180.0,
                                                                           90.3 * 3.14159 / 180.0,
                                                                           -186.79 * 3.14159 / 180.0,
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
        back_command += "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" % (-96.45 * 3.14159 / 180.0,
                                                                                -80.16 * 3.14159 / 180.0,
                                                                                -127.74 * 3.14159 / 180.0,
                                                                                -62.16 * 3.14159 / 180.0,
                                                                                90.3 * 3.14159 / 180.0,
                                                                                -186.79 * 3.14159 / 180.0,
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

        # predict location
        image, uv, precision, theta = self.args[0].locate_object(color_image)
        image_publisher.setData(image)

        xyz, avoid_z = robot_arm.uvd2xyz(uv[0], uv[1], info[0], camera.get_depth_scale())
        # print(uv)
        print(xyz, avoid_z)  #
        x, y, z = xyz[0], xyz[1], avoid_z + 0.195
        if z < 0.27:
            z2 = 0.27
        elif z > 0.65:
            z2 = 0.65
        else:
            z2 = z
        z1 = 0.54
        angle = theta
        print(z2)

        Rx, Ry, Rz = robot_arm.rpy2rotation(3.14, 0, angle)
        self.pick([robot_arm, robot_gripper], [[x, y, z1, Rx, Ry, Rz], [x, y, z2, Rx, Ry, Rz]],
                  [x, y, z2, Rx, Ry, Rz])

        # success label judgement
        time.sleep(0.5)
        color_image2, _ = camera.getImage()
        success_label, imagegray = label.success_label(color_image, color_image2)

        if success_label == 1:
            # robot_gripper.openGripper()
            # time.sleep(0.5)
            # robot_gripper.closeGripper()
            self.place(robot=[robot_arm, robot_gripper])
            return 1, precision, [uv[0], uv[1], x, y, z, angle]
        return 0, precision, [uv[0], uv[1], x, y, z, angle]
            # robot_arm.goHome()

    def task_display(self):
        # parameters initial
        r_camera = self.perception_system['Recorder']
        video_folder = _root_path + '/Data/' + SAVE_NAME + '/video/'
        if not os.path.exists(video_folder):
            os.makedirs(video_folder)

        predictor = Predictor()
        self.args.append(predictor)

        robot_arm, robot_gripper = self.manipulation_system['Arm'], self.manipulation_system['End-effector']
        robot_gripper.closeGripper()
        robot_arm.goHome()

        counter1 = 0.0
        counter2 = 0.0
        counter3 = 0.0
        precision = -1

        for i in range(50):
            recorder = VideoRecorder(camera=r_camera)
            recorder.video_dir = video_folder+str(i)+'.avi'
            thread.start_new_thread(recorder.start, ())

            print('Subtask: ', i)
            successful_label, probability, d = self.subtask_display()
            counter1 += successful_label
            if probability >= 0.5:
                counter2 += 1
                if successful_label == 1:
                    counter3 += 1
                precision = counter3 / counter2
            print("label: %f, probability: %f, grasp successful rate: %f, precision: %f" % (successful_label,
                                                                                            probability,
                                                                                            counter1 / (i+1),
                                                                                            precision))
            graspdata_publisher.setData([d[0], d[1], d[2], d[3], d[4]], d[5], successful_label, probability,
                                        counter1 / (i+1), precision)

            recorder.stop()
            del recorder
            gc.collect()
            time.sleep(1)
            if (i+1)%5==0:
                raw_input('waiting...')
