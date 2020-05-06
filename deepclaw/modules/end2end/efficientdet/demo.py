"""
Waste Sorting Demo
Please Download the pretrained weight from Baidu Pan and put it under the folder /weights/
"""

import glob, cv2, time

from deepclaw.modules.end2end.efficientdet.efficientdet_predictor import efficientdet
detector = efficientdet(0, 'weights/complex/efficientdet-d0_40_60000.pth', num_classes=204)        

# from deepclaw.modules.end2end.efficientdet.efficientdet_predictor_client import efficientdet
# detector = efficientdet(0, 'deepclaw/modules/end2end/efficientdet/weights/complex/efficientdet-d0_40_60000.pth', num_classes=204)        

imgs_path = glob.glob('test/*.png')
ori_imgs = [cv2.imread(img_path) for img_path in imgs_path]

for i in range(len(ori_imgs)):
    print("Predict image %s"%imgs_path[i])
    image = ori_imgs[i]
    t0 = time.time()
    out = detector.run(image)
    print("Prediction time cost: %s"%(time.time()-t0))
    detector.display(out, [image], f'test_out/img_inferred_d0_{i}.jpg')