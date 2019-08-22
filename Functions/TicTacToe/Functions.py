import random
import cv2
import time
import numpy as np
from math import pi
import os
import sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)
print(root_path)

from ClassicalAlgorithms.CVAlgorithm import CVAlgorithm
from TicTacToe.TicTacToeGame import *
CA = CVAlgorithm()

def pieces_detect(color_image, type):
    if type == 'O':
        gray = CA.color2gray(color_image)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
                                   maxRadius=25)
        uvr = []
        if circles is not None:
            circles = np.uint16(np.around(circles))  # shape (1, n, 3)
            for i in range(circles.shape[1]):
                u, v, r = circles[0, i]
                uvr.append([u, v, r])
                cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
            return True, color_image, uvr
        return False, color_image, uvr
    elif type == 'X':
        gray = CA.color2gray(color_image)
        # gray = CA.color2binary(color_image, threshold=200, model=cv2.THRESH_BINARY_INV)
        o_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=22,
                                   maxRadius=25)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=28,
                                     maxRadius=35)
        uvr = []
        if circles is not None and o_circles is not None:
            o_circles = np.uint16(np.around(o_circles))  # shape (1, n, 3)
            circles = np.uint16(np.around(circles))  # shape (1, n, 3)
            for i in range(circles.shape[1]):
                fff = True
                u, v, r = circles[0, i]
                for j in range(o_circles.shape[1]):
                    uo, vo, ro = o_circles[0, j]
                    print(uo, vo, ro, u, v, r)
                    if (np.abs(u-uo)+np.abs(v-vo)) <= 20:
                        fff = False
                        break
                if fff:
                    uvr.append([u, v, r])
                    cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)

            return True, color_image, uvr
        return False, color_image, uvr
    elif type=="ALL":
        gray = CA.color2gray(color_image)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=12,
                                   maxRadius=14)
        uvr = []
        if circles is not None:
            circles = np.uint16(np.around(circles))  # shape (1, n, 3)
            for i in range(circles.shape[1]):
                u, v, r = circles[0, i]
                uvr.append([u, v, r])
                cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
            return True, color_image, uvr
        return False, color_image, uvr

# def board_detect(color_background):
#     # print(root_path+'/Data/TicTacToe/chessboard.png')
#     template = cv2.imread(os.path.dirname(root_path)+'/Data/TicTacToe/chessboard.png', 0)
#     _, top_left, bottom_right = CA.match_template(color_background[200:900, 100:800], template)
#     tlu, tlv = top_left
#     bru, brv = bottom_right
#     delta_w, delta_h = int(float(bru - tlu) / 3), int(float(brv - tlv) / 3)
#     board_pix = np.array([[tlu + delta_w / 2, tlv + delta_h / 2], [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2],
#                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2],
#                           [tlu + delta_w / 2, tlv + delta_h / 2 + delta_h],
#                           [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + delta_h],
#                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + delta_h],
#                           [tlu + delta_w / 2, tlv + delta_h / 2 + 2 * delta_h],
#                           [tlu + delta_w / 2 + delta_w, tlv + delta_h / 2 + 2 * delta_h],
#                           [tlu + delta_w / 2 + 2 * delta_w, tlv + delta_h / 2 + 2 * delta_h]])
#     for pix in board_pix:
#         cv2.circle(color_background, (pix[0], pix[1]), 3, (0, 0, 255), -1)
#     return color_background, board_pix, top_left, bottom_right

def board_detect(camera):
    centers = []
    while len(centers)!=4:
        color_background, depth_image, image_L, image_R = camera.getImage()
        contours = CA.find_contours(color_background, threshold=50)
        contours = CA.contours_area_filter(contours, 4000, 4500)
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

def search_pieces(monitor, camera, board_region, type):
    # Picking
    top_left, bottom_right = board_region
    min_u, min_v = top_left
    max_u, max_v = bottom_right
    flag = False
    while flag == False:
        print('searching available pieces...')
        color_image, depth_image, image_L, image_R = camera.getImage()
        f, color_image, uvr = pieces_detect(color_image, type)
        # monitor.setData(color_image)
        # cv2.imshow('pieces', color_image)
        # cv2.waitKey(1)
        if f:
            for proposed_uv in uvr:
                u, v = proposed_uv[0], proposed_uv[1]
                if min_u <= u and u <= max_u and min_v <= v and v <= max_v:
                    print(u, v)
                else:
                    flag = True
                    break
    return u, v, 0.0

def detect_person_move(color_image, background, game, checkerboard_pix):
    # should be rewritten, return 'False/True, -1/0~8'
    for i in range(9):
        u, v = checkerboard_pix[i]
        tem = sum(np.abs(
            np.mean(background[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3), axis=0) - np.mean(
                color_image[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(20 * 20, 3), axis=0)) > 60)
        print(np.abs(
            np.mean(background[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3), axis=0) - np.mean(
                color_image[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(20 * 20, 3), axis=0)))
        if tem > 0:
            if game.board[i] == " ":
                person_move = i
                print("Detected person_move is at : %s" % person_move)
                return True, person_move
    print("Fail to detect the person move!")
    return False, -1

def board_scan_once(color_image, background, checkerboard_pix):
    flag, color_image, urv = pieces_detect(color_image, 'O')
    # cv2.imshow('circle', color_image)
    # cv2.waitKey(1)
    judgment = []
    for i in range(9):
        u, v = checkerboard_pix[i]
        ltem = sum(np.abs(np.mean(background[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3), axis=0) -
                          np.mean(color_image[v - 10:v + 10, u - 10:u + 10].astype(float).reshape(400, 3),
                                  axis=0)) > 50)
        if ltem > 0:
            ltem = 4
            for circle in urv:
                # print(np.abs(u-circle[0])+np.abs(v-circle[1]))
                if np.abs(u - circle[0]) + np.abs(v - circle[1]) < 20:
                    ltem = 5
                    break
        judgment.append(ltem)
    return judgment

def board_scan(camera, background, board_pix):
    circles_detect = np.zeros(9, dtype='int')
    x_detect = np.zeros(9, dtype='int')
    results = np.zeros(9, dtype='int')

    for i in range(15):
        color_image, depth_image, image_L, image_R = camera.getImage()
        tem_judgment = board_scan_once(color_image, background, board_pix)
        for j in range(9):
            if tem_judgment[j] == 5:
                circles_detect[j] += 1
            if tem_judgment[j] == 4:
                x_detect[j] += 1

    results[circles_detect >= 7] = 5
    results[x_detect >= 7] = 4
    return results

def update_board(game, judgment, ai_move, type):
    if type=='O':
        if judgment[ai_move] != 5:
            print("updating board...")
            game.board[ai_move] = " "
            for i in range(9):
                if game.board[i] == " " and judgment[i] == 5:
                    game.makeMove(i, "O")
                    break
            game.show()
    elif type=='X':
        if judgment[ai_move] != 4:
            print("updating board...")
            game.board[ai_move] = " "
            for i in range(9):
                if game.board[i] == " " and judgment[i] == 4:
                    game.makeMove(i, "X")
                    break
            game.show()
