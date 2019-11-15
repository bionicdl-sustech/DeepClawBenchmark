import cv2
import os
import sys
import copy
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)


def color_image_vis(data, hold_on=1, title="color_image"):
    cv2.imshow(title, data)
    cv2.waitKey(hold_on)


def pcl_vis(data, hold_on=1, title="color_image"):
    pass


def depth_image_vis(data, hold_on=1, title="depth_image"):
    pass


def localization_vis(data, results):
    if "color_image" in data:
        color_image = copy.deepcopy(data["color_image"])
        bounding_boxes = results["bounding_boxes"]
        mask = results["mask"]
        centers = results["centers"]
        if mask is not None:
            c = 0
            for i in set(mask):
                c += 1
                if i != -1:
                    mask_i = np.zeros(color_image.shape)
                    mask_i[mask[:, :] == i] = [int(255.0/c), int(255.0/c), int(255.0/c)]
                    color_image = cv2.add(color_image, mask_i)

        if bounding_boxes is not None:
            for bounding_box in bounding_boxes:
                point = bounding_box[0]
                size = bounding_box[1]
                cv2.rectangle(color_image, (point[0], point[1]), (point[0]+size[0], point[1]+size[1]),
                              [0, 0, 0], 2)

        if centers is not None:
            for center in centers:
                cv2.circle(color_image, center, 3, [255, 255, 255], -1)

        return color_image


def grasp_planning_vis(data, results):
    if "color_image" in data:
        color_image = copy.deepcopy(data["color_image"])
        grasp_pose = results["grasp_pose"]
        x, y, z, roll, pitch, yaw = grasp_pose[0], grasp_pose[1], grasp_pose[2], \
                                    grasp_pose[3], grasp_pose[4], grasp_pose[5]
        cv2.ellipse(color_image, (x, y), (10, 6), roll, 0, 360, (255, 255, 255))
        return color_image
