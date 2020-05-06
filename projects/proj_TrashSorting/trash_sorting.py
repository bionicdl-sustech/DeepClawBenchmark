from deepclaw.modules.calibration.EyeOnBase import load_calibration_matrix
from deepclaw.modules.end2end.efficientdet.efficientdet_predictor import efficientdet
import numpy as np
import cv2, time

class TrashSorting(object):
    def __init__(self):
        pass

    def seg_recog(self, image):
        detector = efficientdet(0, '../../deepclaw/modules/end2end/efficientdet/weights/complex/efficientdet-d0_40_60000.pth', num_classes=204)
        preds = detector.run(image[125:850,650:1375,:])
        i = 0
        if len(preds[i]['rois']) == 0:
            print("Fail to detect any object!")
            return False

        for j in range(1):
            (x1, y1, x2, y2) = preds[0]['rois'][j].astype(np.int)
            obj = detector.obj_list[preds[0]['class_ids'][j]]
            score = float(preds[0]['scores'][j])

        return [x1 + 650, y1 + 125, x2 + 650, y2 + 125, obj, score]

    def grasp_planning(self, image):
        pass

    def motion_planning(self, pick_pose, place_pose, robot = None, gripper = None):
        x, y, z, Rx, Ry, Rz = pick_pose
        p1 = [x, y, 0.41, Rx, Ry, Rz]
        p2 = [x, y, 0.27 , Rx, Ry, Rz]
        if robot is not None and gripper is not None:
            robot.move_ps([p1,p2])
            gripper.close_gripper()
            time.sleep(1)
            robot.move_p([x, y, 0.45 , 0, -3.14, 0])
            robot.go_home()
            gripper.open_gripper()
            return True
        return False


    