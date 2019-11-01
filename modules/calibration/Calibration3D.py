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
