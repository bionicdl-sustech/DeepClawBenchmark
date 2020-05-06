"""
Waste Sorting
"""
from deepclaw.driver.arms.UR10eController import UR10eController
from deepclaw.driver.grippers.handE_controller.gripper_controller import HandEController
from deepclaw.driver.sensors.camera.Realsense import Realsense
from deepclaw.modules.calibration.EyeOnBase import load_calibration_matrix
from trash_sorting import TrashSorting
import numpy as np
import cv2, time

# Initate robot arm and gripper
robot = UR10eController('../../configs/robcell-ur10e-hande-d435/ur10e.yaml')
gripper = HandEController()
robot.go_home()
gripper.active_gripper()

# Initiate Camera
camera = Realsense('../../configs/robcell-ur10e-hande-d435/d435.yaml')
intrinsics = camera.get_intrinsics()
H = load_calibration_matrix('../../configs/robcell-ur10e-hande-d435/ur10e-realsense.npz')

# Initialte task
task = TrashSorting()
cv2.namedWindow("Prediction", cv2.WINDOW_AUTOSIZE)
time.sleep(1)

# Start task
for i in range(2):

    # 0. capture image
    frame = camera.get_frame()
    image = frame.color_image[0]
    image_depth = frame.depth_image[0]

    # 1. detect object and display results
    x1, y1, x2, y2, obj, score = task.seg_recog(image)
    ret = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2)
    ret = cv2.putText(image, '{}, {:.3f}'.format(obj, score), (x1+5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1)
    cv2.imshow('Prediction', image)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
    
    # 2. obtain pick pose by applying handeye transformation
    u = int((x1 + x2)/2)
    v = int((y1 + y2)/2)
    z = image_depth[v, u] if image_depth[v, u]!=0 else 0.96
    x = (u-intrinsics[2])/intrinsics[0] * z # (u-cx/fx) z
    y = (v-intrinsics[3])/intrinsics[1] * z # (v-cy/fy) z
    pick_position = np.matmul(H, np.array([[x], [y], [z], [1]]))
    Rx, Ry, Rz = [0, -3.14, 0] if abs(y1-y2)<abs(x1-x2) else [2.262, 2.18, 0]
    pick_pose = [pick_position[0,0], pick_position[1,0], 0.27 , Rx, Ry, Rz]
    
    # 3. motion planning and execution
    task.motion_planning(pick_pose, robot._home_pose, robot, gripper)

cv2.destroyAllWindows()