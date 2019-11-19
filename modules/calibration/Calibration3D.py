import numpy as np
import cv2


def image_callback(color_image, depth_image, depth_scale, intrinsics):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) * depth_scale
        print("Found checkerboard, Z = ", checkerboard_z)
        checkerboard_x = np.multiply(checkerboard_pix[0] - intrinsics.ppx, checkerboard_z / intrinsics.fx)  # 1920, 1080
        checkerboard_y = np.multiply(checkerboard_pix[1] - intrinsics.ppy, checkerboard_z / intrinsics.fy)  # 1920, 1080
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x,checkerboard_y,checkerboard_z])
            return observed_pt
    return []


def calibrating3d(arm, camera, calibration_cfg):
    initial_pose = calibration_cfg['initial_pose']
    x_step = calibration_cfg['x_step_length']
    y_step = calibration_cfg['y_step_length']
    z_step = calibration_cfg['z_step_length']

    arm.move_j(initial_pose)
    x = initial_pose[0]
    y = initial_pose[1]
    z = initial_pose[2]

    observed_pts = []
    measured_pts = []
    for i in range(4):
        for j in range(4):
            for k in range(4):
                arm.move_j([x + x_step * i, y + y_step * j, z + z_step * k,
                            initial_pose[3], initial_pose[4], initial_pose[5]])
                color_image, info = camera.getImage()
                depth_image = info[0]
                observed_pt = image_callback(color_image, depth_image, camera.get_depth_scale(), camera.getIntrinsics())
                measured_pt = [x + x_step * i, y + y_step * j, z + z_step * k + 0.17]
                if len(observed_pt) != 0:
                    observed_pts.append(observed_pt)
                    measured_pts.append(measured_pt)
    np.savez(calibration_cfg["CALIBRATION_DIR"], observed_pts, measured_pts)
