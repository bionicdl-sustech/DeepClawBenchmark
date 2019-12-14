from __future__ import print_function
import math
from math import cos, sin, acos, pi
# pi = 3.1415926
import numpy as np
from scipy.spatial.transform import Rotation as R
class manipulation_planner():
    def __init__(self, home_location, home_orientation, heave_height):
        self.home_location = home_location
        self.home_orientation = home_orientation
        self.heave_height = heave_height

    # def e2r(self, x, y, z, axes='xyz'):
    #     # y -> z -> x
    #     c1 = cos(y / 2)
    #     c2 = cos(z / 2)
    #     c3 = cos(x / 2)
    #     s1 = sin(y / 2)
    #     s2 = sin(z / 2)
    #     s3 = sin(x / 2)
    #     angle = 2 * acos(c1*c2*c3 - s1*s2*s3)
    #     x = s1*s2*c3 + c1*c2*s3
    #     y = s1*c2*c3 + c1*s2*s3
    #     z = c1*s2*c3 - s1*c2*s3
    #     return x*angle, y*angle, z*angle

    def e2r(self,r,p,y,axes='xyz'):
        r = R.from_euler(axes, [r,p,y], degrees=False)
        Rx, Ry, Rz = r.as_rotvec()
        return Rx, Ry, Rz

    def manipulate_piece(self, which_piece, board_location, board_angle, piece_location, piece_angle, mani_step1_distance, mani_step2_distance):

        # manipulation_flow = [[self.home_location, self.home_orientation, 'release']]
        manipulation_flow = []

        if which_piece == 0:
            pick_angle = self.e2r(0, pi, piece_angle)
            place_angle = self.e2r(0, pi, board_angle)
        elif which_piece == 1:
            pick_angle = self.e2r(0, pi, piece_angle)
            place_angle = self.e2r(-10*pi/180 , pi, board_angle)
        elif which_piece == 2:
            pick_angle = self.e2r(0, pi, piece_angle - pi/4)
            place_angle = self.e2r(-10*pi/180 , pi, board_angle - pi/4)
        elif which_piece == 3:
            pick_angle = self.e2r(0, pi, piece_angle + pi/2)
            place_angle = self.e2r(-10*pi/180 , pi, board_angle + pi/2)
        else:
            raise Exception('param \'which_piece\' error!')


        # 1 move above piece
        location = [piece_location[0], piece_location[1], piece_location[2] + self.heave_height]
        rotation_angle = pick_angle
        suction_cup = 'suck'
        manipulation_flow = manipulation_flow + [[location, rotation_angle, suction_cup]]

        # 2 pick
        location = piece_location
        suction_cup = 'none'
        manipulation_flow = manipulation_flow + [[location, rotation_angle, suction_cup]]

        # 3 leave
        location = [piece_location[0], piece_location[1], piece_location[2] + self.heave_height]
        suction_cup = 'none'
        manipulation_flow = manipulation_flow + [[location, rotation_angle, suction_cup]]

        # 4 move above board
        if which_piece == 0:
            location = [board_location[0],
                        board_location[1],
                        board_location[2] + self.heave_height]
            rotation_angle = place_angle

        elif which_piece == 1:
            location = [board_location[0] + 20 * math.sin(board_angle),
                        board_location[1] + 20 * math.cos(board_angle),
                        board_location[2] + self.heave_height]
            rotation_angle = place_angle

        elif which_piece == 2:
            location = [board_location[0] - 25 * math.sin(board_angle) + 25 * math.cos(board_angle),
                        board_location[1] - 25 * math.cos(board_angle) - 25 * math.sin(board_angle),
                        board_location[2] + self.heave_height]
            rotation_angle = place_angle

        elif which_piece == 3:
            location = [board_location[0] + 20 * math.cos(board_angle),
                        board_location[1] - 20 * math.sin(board_angle),
                        board_location[2] + self.heave_height]
            rotation_angle = place_angle

        suction_cup = 'none'
        manipulation_flow = manipulation_flow + [[[location[0], location[1], location[2]], rotation_angle, suction_cup]]

        # 5 descend
        if which_piece == 0:
            location = board_location

        elif which_piece == 1:
            location[2] = board_location[2]

        elif which_piece == 2:
            location[2] = location[2] - self.heave_height

        elif which_piece == 3:
            location[2] = location[2] - self.heave_height


        suction_cup = 'none'

        manipulation_flow = manipulation_flow + [[location, rotation_angle, suction_cup]]

        # 6 mani_step1

        if which_piece == 0:
            new_board_location = [board_location[0] - mani_step1_distance * math.sin(board_angle),
                                  board_location[1] - mani_step2_distance * math.cos(board_angle),
                                  board_location[2]]
            location = new_board_location

        elif which_piece == 1:
            location = [location[0] - mani_step1_distance * math.sin(board_angle + pi/2),
                        location[1] - mani_step1_distance * math.cos(board_angle + pi/2),
                        location[2]]

        elif which_piece == 2:
            location = [location[0] + mani_step2_distance * math.cos(board_angle),
                        location[1] - mani_step1_distance * math.sin(board_angle),
                        location[2]]

        elif which_piece == 3:
            location = [location[0] + mani_step1_distance * math.sin(board_angle),
                        location[1] + mani_step1_distance * math.cos(board_angle),
                        board_location[2]]

        suction_cup = 'none'
        manipulation_flow = manipulation_flow + [[location, rotation_angle, suction_cup]]

        # 7 mani_step2
        if which_piece == 0:
            new_board_location = [new_board_location[0] - mani_step1_distance * math.cos(board_angle),
                                  new_board_location[1] + mani_step1_distance * math.sin(board_angle),
                                  new_board_location[2]]
            location = new_board_location

        elif which_piece == 1:
            location = [location[0] + mani_step2_distance * math.sin(board_angle + pi),
                        location[1] + mani_step2_distance * math.cos(board_angle + pi),
                        location[2]]

        elif which_piece == 2:
            location = [location[0] - mani_step2_distance * math.sin(board_angle),
                        location[1] - mani_step2_distance * math.cos(board_angle),
                        location[2]]

        elif which_piece == 3:
            location = [location[0] - mani_step2_distance * math.cos(board_angle),
                        location[1] + mani_step2_distance * math.sin(board_angle),
                        location[2]]

        suction_cup = 'release'
        manipulation_flow = manipulation_flow + [[[location[0], location[1], location[2]], rotation_angle, suction_cup]]

        # 8 leave
        if which_piece == 0:
            location = [new_board_location[0], new_board_location[1], new_board_location[2] + self.heave_height]

        elif which_piece == 1:
            location[2] = location[2] + self.heave_height

        elif which_piece == 2:
            location[2] = location[2] + self.heave_height

        elif which_piece == 3:
            location[2] = location[2] + self.heave_height

        suction_cup = 'none'
        manipulation_flow = manipulation_flow + [[location, rotation_angle, suction_cup]]

        # 9 home
        # manipulation_flow = manipulation_flow + [[self.home_location, self.home_orientation, 'none']]

        return manipulation_flow
if __name__ == '__main__':
    p = manipulation_planner(0,0,0)
    print(p.e2r(0, 0, pi, 'xyz'))
