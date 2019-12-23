import numpy as np
import time
import cv2
import cv2.aruco as aruco
import sys
import os
from scipy.spatial.transform import Rotation as R
_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

'''
def image_callback(color_image, depth_image, intrinsics):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    fx = intrinsics[0]
    fy = intrinsics[1]
    cx = intrinsics[2]
    cy = intrinsics[3]

    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) / 1000.0
        checkerboard_x = np.multiply(checkerboard_pix[0] - cx, checkerboard_z / fx)  # 1920, 1080
        checkerboard_y = np.multiply(checkerboard_pix[1] - cy, checkerboard_z / fy)  # 1920, 1080
        print("Found checkerboard, X,Y,Z = ", [checkerboard_x, checkerboard_y, checkerboard_z])
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x, checkerboard_y, checkerboard_z])
            return observed_pt
    return []
'''
def image_callback(color_image, depth_image, intrinsics, calibration_cfg):
    if calibration_cfg['checkerboard_type'] == 'aruco':
        aruco_size = calibration_cfg['aruco_settings']['aruco_size']
        aruco_type = calibration_cfg['aruco_settings']['aruco_type']
        if aruco_type == 'original':
            aruco_dict = aruco.DICT_ARUCO_ORIGINAL
        elif aruco_type == '4x4':
            aruco_dict = aruco.DICT_4X4_250
        elif aruco_type == '5x5':
            aruco_dict = aruco.DICT_5X5_250
        elif aruco_type == '6x6':
            aruco_dict = aruco.DICT_6X6_250
        elif aruco_type == '7x7':
            aruco_dict = aruco.DICT_7X7_250
        else:
            raise ValueError('aruco_type')
        mtx = np.array([
            [intrinsics[0],0,intrinsics[2]],
            [0,intrinsics[1],intrinsics[3]],
            [0,0,1]])
        dist = np.zeros((1,5))
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco_dict)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:         
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, aruco_size, mtx, dist)
            
            for i in range(rvec.shape[0]):
                aruco.drawAxis(color_image, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                aruco.drawDetectedMarkers(color_image, corners)
                cv2.imshow('img',color_image)
                cv2.waitKey(1)
            
            return tvec[0,0,:]
        else:
            return []

    elif calibration_cfg['checkerboard_type'] == 'chessboard':
        checkerboard_size = (3, 3)
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        fx = intrinsics[0]
        fy = intrinsics[1]
        cx = intrinsics[2]
        cy = intrinsics[3]

        gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
        if checkerboard_found:
            corners_refined = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

            # Get observed checkerboard center 3D point in camera space
            checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
            checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) / 1000.0
            checkerboard_x = np.multiply(checkerboard_pix[0] - cx, checkerboard_z / fx)  # 1920, 1080
            checkerboard_y = np.multiply(checkerboard_pix[1] - cy, checkerboard_z / fy)  # 1920, 1080
            print("Found checkerboard, X,Y,Z = ", [checkerboard_x, checkerboard_y, checkerboard_z])
            if checkerboard_z > 0:
                # Save calibration point and observed checkerboard center
                observed_pt = np.array([checkerboard_x, checkerboard_y, checkerboard_z])
                return observed_pt
        return []
    else:
        raise ValueError(" checkerboard_type: 'aruco' or 'chessboard' ")

def calibrating3d(arm, camera, calibration_cfg):
    initial_pose = calibration_cfg['initial_position']
    x_step = calibration_cfg['x_stride']
    y_step = calibration_cfg['y_stride']
    z_step = calibration_cfg['z_stride']

    arm.move_p(initial_pose)
    x = initial_pose[0]
    y = initial_pose[1]
    z = initial_pose[2]

    observed_pts = []
    measured_pts = []

    cube_size = calibration_cfg["cube_size"]
    for i in range(cube_size):
        for j in range(cube_size):
            for k in range(cube_size):
                arm.move_p([x + x_step * i, y + y_step * j, z + z_step * k,
                            initial_pose[3], initial_pose[4], initial_pose[5]],v=0.3)
                #raw_input("waiting...")
                time.sleep(0.5)
                frame = camera.get_frame()

                color_image = frame.color_image[0]
                depth_image = frame.depth_image[0]
                observed_pt = image_callback(color_image, depth_image, camera.get_intrinsics(),calibration_cfg)
                #print(observed_pt)
                # r = R.from_euler('xyz', [initial_pose[3],initial_pose[4],initial_pose[5]], degrees=False)
                # EE_H = np.hstack((r.as_dcm(),np.array([[x + x_step * i], [y + y_step * j], [z + z_step * k]])))
                # EE_H = np.vstack((EE_H,np.array([0,0,0,1])))
                # p = np.hstack((np.array(calibration_cfg["OFFSET"])*1000,1))
                #print(EE_H,p.T)
                # measured_pt = np.dot(EE_H,p.T)
                # measured_pt = measured_pt[0:3]
                measured_pt = [x + x_step * i + calibration_cfg["OFFSET"][0], y + y_step * j + calibration_cfg["OFFSET"][1], z + z_step * k + calibration_cfg["OFFSET"][2]]
                print(measured_pt, observed_pt)
                if len(observed_pt) != 0:
                    observed_pts.append(observed_pt)
                    measured_pts.append(measured_pt)
    np.savez(os.path.dirname(_root_path)+calibration_cfg["CALIBRATION_DIR"], observed_pts, measured_pts)
'''

def image_callback(color_image, depth_image, intrinsics):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    fx = intrinsics[0]
    fy = intrinsics[1]
    cx = intrinsics[2]
    cy = intrinsics[3]

    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) / 1000.0
        checkerboard_x = np.multiply(checkerboard_pix[0] - cx, checkerboard_z / fx)  # 1920, 1080
        checkerboard_y = np.multiply(checkerboard_pix[1] - cy, checkerboard_z / fy)  # 1920, 1080
        print("Found checkerboard, X,Y,Z = ", [checkerboard_x, checkerboard_y, checkerboard_z])
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x, checkerboard_y, checkerboard_z])
            return observed_pt
    return []


def calibrating3d(arm, camera, calibration_cfg):
    initial_pose = calibration_cfg['initial_position']
    x_step = calibration_cfg['x_stride']
    y_step = calibration_cfg['y_stride']
    z_step = calibration_cfg['z_stride']

    arm.move_p(initial_pose)
    x = initial_pose[0]
    y = initial_pose[1]
    z = initial_pose[2]

    observed_pts = []
    measured_pts = []
    for i in range(4):
        for j in range(4):
            for k in range(4):
                arm.move_p([x + x_step * i, y + y_step * j, z + z_step * k,
                            initial_pose[3], initial_pose[4], initial_pose[5]])
                # time.sleep(0.5)
                frame = camera.get_frame()
                color_image = frame.color_image[0]
                depth_image = frame.depth_image[0]
                observed_pt = image_callback(color_image, depth_image, camera.get_intrinsics())
                measured_pt = [x + x_step * i + calibration_cfg["OFFSET"][0], y + y_step * j + calibration_cfg["OFFSET"][1], z + z_step * k + calibration_cfg["OFFSET"][2]]
                print(measured_pt)
                if len(observed_pt) != 0:
                    observed_pts.append(observed_pt)
                    measured_pts.append(measured_pt)
    np.savez(os.path.dirname(_root_path)+calibration_cfg["CALIBRATION_DIR"], observed_pts, measured_pts)
'''