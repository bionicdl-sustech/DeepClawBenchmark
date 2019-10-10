import random
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

SAVE_NAME = 'training_data_59'

image_publisher = ImagePublisher('image_pub')
graspdata_publisher = GraspingDataPublisher('grasp_data_pub')

data_collector = Monitor(SAVE_NAME)
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

        time.sleep(0.5)
        color_image, _ = camera.getImage()
        image_publisher.setData(color_image)

        # x = random.uniform(0.370, 0.715)
        # y = random.uniform(-0.542, -0.092)
        u = random.uniform(490, 850)
        v = random.uniform(135, 550)
        angel = random.uniform(-1.57, 1.57)

        _, info = camera.getImage()
        xyz, avoid_z = robot_arm.uvd2xyz(u, v, info[0], camera.get_depth_scale())
        print(xyz[2], avoid_z) #
        x, y, z = xyz[0], xyz[1], avoid_z + 0.195
        place_z = 0.54
        if z < 0.27:
            pick_z = 0.27
        elif z > 0.65:
            pick_z = 0.65
        else:
            pick_z = z
        print(pick_z)

        goal_pose = [[x, y, place_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        robot_gripper.openGripper()
        goal_pose = [[x, y, pick_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        robot_gripper.closeGripper()
        goal_pose = [[x, y, place_z], [3.14, 0, angel]]
        robot_arm.move(goal_pose)
        robot_arm.goHome()

        time.sleep(0.5)
        color_image2, _ = camera.getImage()
        success_label, imagegray = label.success_label(color_image, color_image2)
        graspdata_publisher.setData([u, v, x, y, pick_z], angel, success_label)

        if success_label == 1:
            u = random.uniform(490, 850)
            v = random.uniform(135, 550)
            angel = random.uniform(-1.57, 1.57)

            _, info = camera.getImage()
            xyz, _ = robot_arm.uvd2xyz(u, v, info[0], camera.get_depth_scale())
            x, y, z = xyz[0], xyz[1], 0.4
            place_z = 0.54
            pick_z = z

            goal_pose = [[x, y, place_z], [3.14, 0, angel]]
            robot_arm.move(goal_pose)
            goal_pose = [[x, y, pick_z], [3.14, 0, angel]]
            robot_arm.move(goal_pose)
            robot_gripper.openGripper()
            robot_arm.goHome()
            robot_gripper.closeGripper()

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
