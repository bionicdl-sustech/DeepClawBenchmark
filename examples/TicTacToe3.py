import os
import sys
import time
import math

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor
from input_output.observers.TicTacToeDataMonitor import TicTacToeDataMonitor
from modules.localization.contour_filter import contour_filter
from modules.recognition.color_recognition import color_recognition
from modules.grasp_planning.random_planner import RandomPlanner
from modules.grasp_planning.minmax_planner import MinMaxPlanner


def calculate_rotation(point, theta):
    x, y = point[0], point[1]
    rotated_x = x * math.cos(theta) - y * math.sin(theta)
    rotated_y = y * math.cos(theta) + x * math.sin(theta)
    return [rotated_x, rotated_y]


def calculate_shift(original_point, relative_point):
    ox, oy = original_point[0], original_point[1]
    rx, ry = relative_point[0], relative_point[1]
    x = ox + rx
    y = oy + ry
    return [int(x), int(y)]


def calculate_relative_point(reference_point, length, theta):
    original_point = [length, 0]
    rotated_point = calculate_rotation(original_point, theta)
    relative_point = calculate_shift(rotated_point, reference_point)
    return relative_point


def find_block_centers(center, theta, length=30):
    centers = [center]
    for i in range(8):
        angle = -((i * math.pi / 4) + theta)
        l = length * (1 - (i%2) + math.sqrt(2) * (i%2))
        centers.append(calculate_relative_point(center, l, angle))
    converted_centers = [[], [], [], [], [], [], [], [], []]
    for i in range(len(centers)):
        index = convert_index(i)
        converted_centers[index] = centers[i]
    return converted_centers


def convert_index(index):
    if index == 0:
        return 4
    elif index == 1:
        return 5
    elif index == 2:
        return 2
    elif index == 3:
        return 1
    elif index == 4:
        return 0
    elif index == 5:
        return 3
    else:
        return index


class TicTacToe3(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(TicTacToe3, self).__init__(perception_system, manipulation_system, is_debug)
        time_s = time.localtime(int(time.time()))
        self.experiment_name = "experiment_" + str(time_s.tm_mon) + str(time_s.tm_mday) + \
                               str(time_s.tm_hour) + str(time_s.tm_min) + str(time_s.tm_sec)
        self.data_path = _root_path+"/data/"+os.path.basename(__file__).split(".")[0]+"/"+self.experiment_name+"/"
        self.args = {"WorkSpace": [[350, 550], [450, 850]], "subtask_counter": 0} # kinect
        # self.args = {"WorkSpace": [[100, 550], [450, 850]]} # realsense
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.data_monitor = TicTacToeDataMonitor("data_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)
        self.publisher.registerObserver(self.data_monitor)

        self.piece_localization_operator = contour_filter(area_threshold=[300, 500])  # kinect
        self.board_localization_operator = contour_filter(area_threshold=[10000, 10300], minAreaBox=True)
        # self.localization_operator = contour_filter(area_threshold=[900, 1050])  # realsense
        self.blue_recognition_operator = color_recognition([100, 43, 46], [124, 255, 255])  # blue
        self.green_recognition_operator = color_recognition([35, 43, 46], [77, 255, 255])  # green
        self.pick_grasp_planner = RandomPlanner([[3.14, 3.14], [0, 0], [0, 0]])
        self.green_grasp_planner = MinMaxPlanner(player="G")
        self.blue_grasp_planner = MinMaxPlanner(player="B")
        self.motion_planner = ''

    def task_display(self):
        # results path
        self.time_monitor.dir = self.data_path
        self.time_monitor.csv_name = self.experiment_name+"_CostTime.csv"
        self.data_monitor.dir = self.data_path
        self.data_monitor.csv_name = self.experiment_name+"_TicTacToeData.csv"

        # sub-task display
        self.arm.go_home()
        self.gripper.open_gripper()
        for i in range(10):
            print("Sutask "+str(i+1)+" displaying...")
            self.args["subtask_counter"] = i
            print("picking...")
            self.subtask_pick_display()
            print("placing...")
            self.subtask_place_display()
            raw_input("waiting...")
        return self.data_path

    def subtask_pick_display(self):
        # subtask data path
        subtask_name = "subtask_" + str(self.args["subtask_counter"])
        path = self.data_path + subtask_name + "/"
        self.image_monitor.dir = path

        # color image pre-process
        frame = self.camera.get_frame()
        color_image = frame.color_image[0]
        depth_image = frame.depth_image[0]
        sub_image = color_image[self.args["WorkSpace"][0][0]:self.args["WorkSpace"][0][1], 
                                self.args["WorkSpace"][1][0]:self.args["WorkSpace"][1][1],
                                :]
        cidata = {"Image": color_image}
        didata = {"Image": depth_image}
        self.image_monitor.img_name = subtask_name + "_rgb_piece.jpg"
        self.publisher.sendData(cidata)
        self.image_monitor.img_name = subtask_name + "_d_piece.jpg"
        self.publisher.sendData(didata)

        # segmentation
        start = time.time()
        bounding_box, mask, centers = self.piece_localization_operator.display(sub_image)
        end = time.time()

        # data process
        tdata = {"Time": [subtask_name+' segmentation_time_sub1', end-start]}
        self.publisher.sendData(tdata)

        # recognition
        start = time.time()
        if self.args["subtask_counter"] % 2 == 0:
            labels, _ = self.blue_recognition_operator.display(centers, sub_image)
        else:
            labels, _ = self.green_recognition_operator.display(centers, sub_image)
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time_sub1', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        for center in centers:
            center[0] = self.args["WorkSpace"][1][0] + center[0]
            center[1] = self.args["WorkSpace"][0][0] + center[1]
        print(centers, labels)
        pose = self.pick_grasp_planner.display(centers, labels)
        if pose is not None:
            xyz, avoidz = self.arm.uvd2xyz(pose[0], pose[1], depth_image, self.camera.get_intrinsics())
            pose[0], pose[1], pose[2] = xyz[0], xyz[1], 0.25
        tdata = {"Time": [subtask_name+' grasp_planning_time_sub1', end - start]}
        self.publisher.sendData(tdata)

        # motion planning
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' motion_planning_time_sub1', end - start]}
        self.publisher.sendData(tdata)

        # execution
        if pose is not None:
            start = time.time()
            self.arm.move_p(pose)
            pose[2] = 0.165
            self.arm.move_p(pose)
            self.gripper.close_gripper()
            end = time.time()
        else:
            start = time.time()
            end = time.time()

        tdata = {"Time": [subtask_name + ' execution_time_sub1', end - start]}
        self.publisher.sendData(tdata)

        tttdata = {"TicTacToeData": [bounding_box, centers, labels, pose]}
        self.publisher.sendData(tttdata)

        self.arm.go_home()
        
    def subtask_place_display(self):
        # subtask data path
        subtask_name = "subtask_" + str(self.args["subtask_counter"])
        path = self.data_path + subtask_name + "/"
        self.image_monitor.dir = path

        # color image pre-process
        frame = self.camera.get_frame()
        color_image = frame.color_image[0]
        depth_image = frame.depth_image[0]
        sub_image = color_image[self.args["WorkSpace"][0][0]:self.args["WorkSpace"][0][1], 
                                self.args["WorkSpace"][1][0]:self.args["WorkSpace"][1][1],
                                :]
        cidata = {"Image": color_image}
        didata = {"Image": depth_image}
        self.image_monitor.img_name = subtask_name + "_rgb_board.jpg"
        self.publisher.sendData(cidata)
        self.image_monitor.img_name = subtask_name + "_d_board.jpg"
        self.publisher.sendData(didata)

        # segmentation
        start = time.time()
        bounding_box, mask, centers = self.board_localization_operator.display(sub_image)
        if len(centers) != 0:
            centers = find_block_centers(centers[0], -bounding_box[0][2] * math.pi / 180)
        end = time.time()

        # data process
        tdata = {"Time": [subtask_name+' segmentation_time_sub2', end-start]}
        self.publisher.sendData(tdata)

        # recognition
        start = time.time()
        b_labels, _ = self.blue_recognition_operator.display(centers, sub_image)
        g_labels, _ = self.green_recognition_operator.display(centers, sub_image)
        labels = []
        for b, g in zip(b_labels, g_labels):
            if b != g:
                if b:
                    labels.append(1)
                elif g:
                    labels.append(2)
            else:
                labels.append(0)
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time_sub2', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        for center in centers:
            center[0] = self.args["WorkSpace"][1][0] + center[0]
            center[1] = self.args["WorkSpace"][0][0] + center[1]
        if self.args["subtask_counter"] % 2 == 0:
            start = time.time()
            pose = self.blue_grasp_planner.display(centers, labels)
            end = time.time()
        else:
            start = time.time()
            pose = self.green_grasp_planner.display(centers, labels)
            end = time.time()
        tdata = {"Time": [subtask_name+' grasp_planning_time_sub2', end - start]}
        self.publisher.sendData(tdata)

        if pose is not None:
            xyz, avoidz = self.arm.uvd2xyz(pose[0], pose[1], depth_image, self.camera.get_intrinsics())
            pose[0], pose[1], pose[2] = xyz[0], xyz[1], 0.25
        # motion planning
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' motion_planning_time_sub2', end - start]}
        self.publisher.sendData(tdata)

        # execution
        if pose is not None:
            start = time.time()
            self.arm.move_p(pose)
            pose[2] = 0.165
            self.arm.move_p(pose)
            self.gripper.open_gripper()
            pose[2] = 0.25
            self.arm.move_p(pose)
            end = time.time()
        else:
            start = time.time()
            end = time.time()

        tdata = {"Time": [subtask_name + ' execution_time_sub2', end - start]}
        self.publisher.sendData(tdata)

        tttdata = {"TicTacToeData": [bounding_box, centers, labels, pose]}
        self.publisher.sendData(tttdata)

        self.arm.go_home()
