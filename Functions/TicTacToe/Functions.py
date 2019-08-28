import random
import cv2
import time
import numpy as np
from math import pi
import os
import sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

from ClassicalAlgorithms.CVAlgorithm import CVAlgorithm
from TicTacToe.TicTacToeGame import *
CA = CVAlgorithm()

def localization_display(data_publisher, camera, last_results, is_debug=False):
    '''Localization'''
    top_left, bottom_right = last_results[0], last_results[1]
    min_u, min_v = top_left
    max_u, max_v = bottom_right
    while True:
        if is_debug:
            print('localizing...')
        color_image, _ = camera.getImage()
        gray = CA.color2gray(color_image)
        # o_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
        #                              maxRadius=25)
        pieces = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=33, maxRadius=38)
        uvr = []
        if pieces is not None:
            pieces = np.uint16(np.around(pieces))  # shape (1, n, 3)
            for i in range(pieces.shape[1]):
                u, v, r = pieces[0, i]
                if u<min_u or max_u<u or v<min_v or max_v<v:
                    uvr.append([u, v, r])
                    if is_debug:
                        cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)

            if is_debug:
                data_publisher.setData(color_image)
                # cv2.imshow('pieces', color_image)
                # cv2.waitKey(0)
            return uvr

def identification_display(data_publisher, camera, last_results, is_debug=False):
    '''Identification'''
    uvr, top_left, bottom_right, piece_type = last_results[0], last_results[1], last_results[2], last_results[3]
    min_u, min_v = top_left
    max_u, max_v = bottom_right
    while True:
        if is_debug:
            print('identifying...')
        color_image, _ = camera.getImage()
        gray = CA.color2gray(color_image)
        o_pieces = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22, maxRadius=28)
        x_uvr, o_uvr = [], []
        if o_pieces is not None:
            o_pieces = np.uint16(np.around(o_pieces))  # shape (1, n, 3)
            for i in range(o_pieces.shape[1]):
                u, v, r = o_pieces[0, i]
                if u<min_u or max_u<u and v<min_v and max_v<v:
                    o_uvr.append([u, v, r])

            for i in range(len(uvr)):
                flag = True
                u, v, r = uvr[i][0], uvr[i][1], uvr[i][2]
                for j in range(len(o_uvr)):
                    uo, vo, ro = o_uvr[j][0], o_uvr[j][1], o_uvr[j][2]
                    if (np.square(u-uo)+np.square(v-vo)) <= 100:
                        flag = False
                        break
                if flag:
                    x_uvr.append([u, v, r])
            if piece_type==0:
                if is_debug:
                    for xxx in x_uvr:
                        u, v, r = xxx[0], xxx[1], xxx[2]
                        cv2.circle(color_image, (u, v), r, (0, 0, 255), 2)
                    data_publisher.setData(color_image)
                    # cv2.imshow('red_pieces', color_image)
                    # cv2.waitKey(0)
                return x_uvr
            else:
                if is_debug:
                    for xxx in o_uvr:
                        u, v, r = xxx[0], xxx[1], xxx[2]
                        cv2.circle(color_image, (u, v), r, (255, 0, 0), 2)
                    data_publisher.setData(color_image)
                    # cv2.imshow('blue_pieces', color_image)
                    # cv2.waitKey(0)
                return o_uvr

def multiple_points_motion_planning(data_publisher, robot, last_results, is_debug=False):
    '''Multiple Points Motion Planning'''
    selected_pieces_uvr, game, board_pix, piece_type = last_results[0], last_results[1], last_results[2], last_results[3]

    pick_num = random.randint(0, len(selected_pieces_uvr)-1)
    pick_u, pick_v = selected_pieces_uvr[pick_num][0], selected_pieces_uvr[pick_num][1]

    if piece_type==0:
        ai_move = make_best_move(game, -1, "X")
        game.makeMove(ai_move, "X")
    else:
        ai_move = make_best_move(game, -1, "O")
        game.makeMove(ai_move, "O")
    place_u, place_v = board_pix[ai_move][0], board_pix[ai_move][1]

    pick_location = robot.calibration_tool.cvt(pick_u, pick_v)
    place_location = robot.calibration_tool.cvt(place_u, place_v)

    return pick_location, place_location

def execution_display(data_publisher, robot, end_effector, last_results, is_debug=False):
    '''Execution'''
    pick_location, place_location = last_results[0], last_results[1]

    robot.movej(pick_location[0], pick_location[1], robot.PICK_Z + 0.01, pi, 0, 0)
    robot.movej(pick_location[0], pick_location[1], robot.PICK_Z, pi, 0, 0)
    end_effector.closeGripper()

    robot.movej(place_location[0], place_location[1], robot.PLACE_Z + 0.01, pi, 0, 0)
    robot.movej(place_location[0], place_location[1], robot.PLACE_Z, pi, 0, 0)
    end_effector.openGripper()
    robot.goHome()


def board_detect(camera):
    centers = []
    while len(centers)!=4:
        print('board detecting...')
        color_background, _ = camera.getImage()
        contours = CA.find_contours(color_background, threshold=50)
        contours = CA.contours_area_filter(contours, 5800, 6800)
        centers = CA.find_contours_center(contours)
        # cv2.drawContours(c, contours, -1, (0, 255, 0), 1)
        cx, cy = 0, 0
        for center in centers:
            # cv2.circle(color_background, (center[0], center[1]), 1, (0, 255, 0), 2)
            cx+=center[0]
            cy+=center[1]
    tlu, tlv = cx/4-100, cy/4-100
    bru, brv = cx/4+100, cy/4+100
    delta_w, delta_h = int(float(bru - tlu) / 3), int(float(brv - tlv) / 3)
    board_pix = np.array([[tlu + delta_w / 2, tlv + delta_h / 2], [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2],
                          [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2],
                          [tlu + delta_w / 2, tlv + delta_h / 2 + delta_h],
                          [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + delta_h],
                          [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + delta_h],
                          [tlu + delta_w / 2, tlv + delta_h / 2 + 2 * delta_h],
                          [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + 2 * delta_h],
                          [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + 2 * delta_h]])
    for pix in board_pix:
        cv2.circle(color_background, (pix[0], pix[1]), 3, (0, 255, 255), -1)
    return color_background, board_pix, (tlu, tlv), (bru, brv)

