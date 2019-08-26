import numpy as np
from math import pi
import cv2
import os
import sys
from PIL import ImageDraw

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

def end2end_display(data_publisher, camera, last_results, is_debug=False):
    '''Localization'''
    Network, croped_image, NUM_BOXES, WIDTH = last_results[0], last_results[1], last_results[2], last_results[3]
    patches, boxes = Network.generate_patches(croped_image, NUM_BOXES, WIDTH)
    candidates_theta, candidates_probability = Network.eval_theta(patches)
    return candidates_theta, candidates_probability, patches, boxes

def multiple_points_motion_planning(data_publisher, manipulation_system, last_results, is_debug=False):
    '''Multiple Points Motion Planning'''
    robot_arm = manipulation_system[0]
    robot_gripper = manipulation_system[1]
    candidates_theta, candidates_probability = last_results[0], last_results[1]
    croped_image, boxes = last_results[2], last_results[3]
    NUM_BOXES, WIDTH, crop_box = last_results[4], last_results[5], last_results[6]

    best_idx = np.argmax(candidates_probability)

    if is_debug:
        draw = ImageDraw.Draw(croped_image, 'RGBA')
        for j in range(len(boxes)):
            x = (boxes[j][0] + boxes[j][2]) / 2
            y = (boxes[j][1] + boxes[j][3]) / 2
            r = candidates_probability[j] * ((croped_image.size[0] - WIDTH) / NUM_BOXES / 2)

            if j == best_idx or candidates_probability[j] > 0.5:
                best_theta = (-1.57 + (candidates_theta[j] + 0.5) * (1.57 / 9))
                draw.ellipse((x - r, y - r, x + r, y + r), (0, 0, 255, 125))
                draw.line([(x - r * np.cos(best_theta), y + r * np.sin(best_theta)),
                           (x + r * np.cos(best_theta), y - r * np.sin(best_theta))],
                          fill=(255, 255, 255, 125), width=10)
        data_publisher.setData(np.array(croped_image))
        # print('save image')
        # croped_image.save('/home/h/result.jpg')

    x = (boxes[best_idx][0] + boxes[best_idx][2]) / 2 + crop_box[1]
    y = (boxes[best_idx][1] + boxes[best_idx][3]) / 2 + crop_box[0]
    # best_theta = (-1.57 + (candidates_theta[best_idx] - 0.5) * (1.57 / 9))
    best_theta = ((candidates_theta[best_idx] - 0.5) * (1.57 / 9))

    pick_location = robot_arm.calibration_tool.cvt(x, y)
    place_location = [robot_arm.HOME_POSE[0][0], robot_arm.HOME_POSE[0][1]]
    return pick_location, place_location, best_theta


def execution_display(data_publisher, manipulation_system, last_results, is_debug=False):
    '''Execution'''
    robot_arm = manipulation_system[0]
    robot_gripper = manipulation_system[1]
    pick_location, place_location, best_theta = last_results[0], last_results[1], last_results[2]

    robot_arm.movej(pick_location[0], pick_location[1], robot_arm.PICK_Z + 0.05, pi, 0, best_theta)
    robot_arm.movej(pick_location[0], pick_location[1], robot_arm.PICK_Z, pi, 0, best_theta)
    robot_gripper.closeGripper()

    robot_arm.movej(place_location[0], place_location[1], robot_arm.PLACE_Z + 0.01, pi, 0, 0)
    robot_arm.movej(place_location[0], place_location[1], robot_arm.PLACE_Z, pi, 0, 0)
    robot_gripper.openGripper()
