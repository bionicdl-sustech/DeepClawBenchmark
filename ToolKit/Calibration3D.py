import numpy as np
import cv2

def image_callback(color_image, depth_image, depth_scale):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)

        sum_z = 0
        error = 0
        ite = 1
        for i in range(40):
            for j in range(40):
                tem_z = depth_image[checkerboard_pix[1]-20+i][checkerboard_pix[0]-20+j]
                if 0 < tem_z < 1000:
                    sum_z += tem_z
                    ite += 1
                else:
                    error += 1
        checkerboard_z = (sum_z / (ite-1)) * depth_scale

        # checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) * depth_scale
        print("Found checkerboard, Z = ", checkerboard_z)
        checkerboard_x = np.multiply(checkerboard_pix[0]-624.79,checkerboard_z/931.69)
        checkerboard_y = np.multiply(checkerboard_pix[1]-360.52,checkerboard_z/931.46)
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x,checkerboard_y,checkerboard_z])

        ## Draw and display the corners
        vis = cv2.drawChessboardCorners(color_image, (1,1), corners_refined[4,:,:], checkerboard_found)
        cv2.imwrite('./color_image_checkerboard_%02d.png' % ite, vis)
        print('error: ', error)
        return observed_pt, vis
