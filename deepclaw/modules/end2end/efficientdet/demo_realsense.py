# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: efficientdet demo
@Author: Fang Wan
@Date: 2020/5/6 13:26
@Description: 
"""

from deepclaw.driver.sensors.camera.Realsense import Realsense
from deepclaw.modules.end2end.efficientdet.efficientdet_predictor import efficientdet
import numpy as np
import cv2, time

camera = Realsense('../../../../configs/robcell-ur10e-hande-d435/d435.yaml')
intrinsics = camera.get_intrinsics()

detector = efficientdet(0, 'weights/complex/efficientdet-d0_40_60000.pth', num_classes=204)

while True:
    frame = camera.get_frame()
    image = frame.color_image[0]
    preds = detector.run([image])
    if len(preds[0]['rois']) == 0:
        print("Fail to detect any object! Lower the score threshold if needed.")
        continue
    for j in range(len(preds[0]['rois'])):
        (x1, y1, x2, y2) = preds[0]['rois'][j].astype(np.int)
        obj = detector.obj_list[preds[0]['class_ids'][j]]
        score = float(preds[0]['scores'][j])
        ret = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2)
        ret = cv2.putText(image, '{}, {:.3f}'.format(obj, score),
                    (x1+5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 255), 1)
    cv2.imshow('Result', image)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cv2.destroyAllWindows()