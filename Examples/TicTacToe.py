import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from Examples.Task import Task
from Functions.TicTacToe.TicTacToeGame import *
from Functions.TicTacToe.Functions import *
from ToolKit.DataCollector import ImagePublisher, TimePublisher, Monitor

image_publisher = ImagePublisher('image_pub')
time_publisher = TimePublisher('time_pub')
data_collector = Monitor('Tic-tac-toe-task5')
image_publisher.registerObserver(data_collector)
time_publisher.registerObserver(data_collector)

class TicTacToeTask(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(TicTacToeTask, self).__init__(perception_system, manipulation_system, is_debug)
        self.args = ''

    def subtask_display(self):
        # args initial
        camera = self.perception_system[0]
        robot_arm,  robot_gripper = self.manipulation_system[0], self.manipulation_system[1]
        game, board_pix, top_left, bottom_right, ite = self.args[0], self.args[1], self.args[2], self.args[3], self.args[4]
        piece_type = ite % 2

        # localization step
        start_time = time.time()
        pieces_uvr = localization_display(image_publisher, camera, [top_left, bottom_right], self.is_debug)
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'localization', end_time - start_time])

        # identification step
        start_time = time.time()
        selected_pieces_uvr = identification_display(image_publisher, camera,
                                                     [pieces_uvr, top_left, bottom_right, piece_type], self.is_debug)
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'identification', end_time - start_time])

        # multiple points motion planning step
        start_time = time.time()
        pick_location, place_location = multiple_points_motion_planning(image_publisher, robot_arm,
                                                                        [selected_pieces_uvr, game, board_pix, piece_type])
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'multiple_points_motion_planning', end_time - start_time])

        # execution step
        start_time = time.time()
        execution_display(image_publisher, robot_arm, robot_gripper, [pick_location, place_location])
        end_time = time.time()
        if self.is_debug:
            time_publisher.setData([str(ite)+'execution', end_time - start_time])

    def task_display(self):
        # game initial
        game = TicTacToe()

        # sensors
        camera = self.perception_system['Camera']

        # board detection
        color_image, board_pix, top_left, bottom_right = board_detect(camera)
        for i in range(20):
            color_image, board_pix, top_left, bottom_right = board_detect(camera)

        if self.is_debug:
            # cv2.imshow('board', color_image)
            # cv2.waitKey(1000)
            image_publisher.setData(color_image)

        ite = 0
        while game.gameOver() == False:
            self.args = [game, board_pix, top_left, bottom_right, ite]
            self.subtask_display()
            ite += 1
            grasp_label = raw_input('grasp label[0/1/2]:')
            if self.is_debug:
                time_publisher.setData([str(ite) + 'grasp label', str(grasp_label)])

