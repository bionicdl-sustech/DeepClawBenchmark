# Author: Fang Wan

"""
Simple Inference Script of EfficientDet-Pytorch
"""
import time

import torch
from torch.backends import cudnn
import cv2
import numpy as np

from .backbone import EfficientDetBackbone
from .efficientdet.utils import BBoxTransform, ClipBoxes
from .utils.utils import preprocess_image, invert_affine, postprocess

class efficientdet(object):
    def __init__(self, compound_coef = 0, weight_path = None, num_classes = 204):
        self.compound_coef = compound_coef
        self.weight_path = weight_path

        if num_classes == 4:
            self.obj_list = ['glass', 'paper', 'metal', 'plastic']
        else:
            self.obj_list = ['seed shell', 'walnut', 'peanut shell', 'edamame shell', 'watermelon seed', 'date pit', 'plum pit', 'apple peel', 'persimmon peel', 'watermelon rind', 'banana peel', 'shaddock peel', 'litchi shell', 'mango peel', 'apple core', 'dried fruit', 'orange peel', 'biscuit', 'bread', 'candy', 'pet food', 'air-dried food', 'candied fruit', 'jerky', 'brewed beverage powder', 'cheese', 'can', 'cake', 'potato chips', 'leaves', 'weeds', 'green plant', 'fresh flowers', 'beans', 'animal organs', 'mung bean rice', 'cereals and processed products', 'shelled without shell', 'shrimp', 'pasta', 'meat', 'whole grains', 'rib', 'chicken', 'chicken bones', 'snail', 'duck', 'fish', 'vegetable root', 'cauliflower', 'mushrooms', 'fish scale', 'seasoning', 'tea residue', 'coffee grounds', 'zongzi', 'animal hoof', 'crayfish', 'crab shell', 'sauce', 'fish bones', 'eggshell', 'Chinese herbal medicine', 'Chinese medicine residue', 'mirror', 'glass product', 'window glass', 'broken glass', 'cosmetic glass bottle', 'glass jars for food and daily necessities', 'vacuum cup', 'glass', 'journal', 'newspaper', 'food packaging box', 'shoe box', 'tetra pack', 'leaflets', 'printer paper', 'paper shopping bag', 'calendar', 'paper courier bags', 'envelope', 'cigarette case', 'cans', 'metallica', 'magnet', 'aluminium product', 'metal bottles', 'metal tools', 'tin can', 'spoon', 'kitchen knife', 'fork', 'pot', 'metal chopsticks', 'USB cable', 'plastic toy', 'mineral water bottle', 'plastic foam', 'plastic package', 'rigid plastic', 'disposable plastic dinnerware', 'electric wire', 'plastic hanger', 'melamine tableware', 'acrylic board', 'PVC tube', 'socket', 'cosmetic plastic bottle', 'basketball', 'football', 'KT board', 'food plastic box', 'cooking oil containers', 'plastic cup', 'plastic bottle', 'disposable lunch box', 'discarded clothes', 'shoe', 'cloth waste', 'school bag', 'bedding', 'quilt', 'silk handkerchief', 'pillow', 'plush toy', 'belt', 'circuit board', 'power bank', 'woodware', 'flash disk', 'lamp bulb', 'energy saving lamp', 'diode', 'button cell', 'cell phone battery', 'nickel--cadmiun battery', 'lithium battery', 'accumulator', 'film', 'photograph', 'nail', 'X-ray film', 'pesticide bottle', 'insecticides and cans', 'candle', 'ink cartridge', 'hair dye shell', 'disinfectant bottle', 'paint bucket', 'drug packaging', 'medicine bottle', 'waste needle tube', 'infusion tube', 'oral liquid bottle', 'eye drops bottle', 'mercurial thermometer', 'mercurial sphygmomanometer', 'capsule', 'tablet', 'solid insecticide', 'sugarcane skin', 'nutshell', 'plasticine', 'hair', 'swab', 'band-aid', 'lipstick', 'pen', 'tissue', 'tape', 'wet tissue', 'water color pen', 'lighter', 'bubble shatter-proof warp', 'durian shell', 'eye black', 'eye shadow', 'hamster sand bath', 'big bone rods', 'old towel', 'bamboo ware', 'chalk', 'disposable mask', 'disposable glove', 'liquid foundation', 'dust', 'nylon products', 'diaper', 'umbrella', 'tape product', 'toothpaste', 'the dog diapers', 'coconut shell', 'powder puff', 'broken dishes', 'ceramic', 'toilet paper', 'cigarette end', 'false eyelashes', 'cat litter', 'toothbrush', 'corn cob']
   
        self.force_input_size = None  # set None to use default size
        self.threshold = 0.5
        self.iou_threshold = 0.1 #The lower, the less overlaid bbox
        self.use_cuda = True
        self.use_float16 = False # keep False, torchvision.nms produces wrong results while use float16
        cudnn.fastest = True
        cudnn.benchmark = True

        # load the model trained from recyclable trash data
        # The model is able to detect single recyclable object in the image
        self.model = EfficientDetBackbone(compound_coef=self.compound_coef, num_classes=len(self.obj_list))
        self.model.load_state_dict(torch.load(self.weight_path))
        self.model.requires_grad_(False)
        self.model.eval()
        print('Loaded weights')
        if self.use_cuda:
            self.model = self.model.cuda()
        if self.use_float16:
            self.model = self.model.half()

    def run(self, image_np):
        # print('==========================Start predicting...')
        t0 = time.time()
        # tf bilinear interpolation is different from any other's, just make do
        image = [np.array(image_np)]
        input_sizes = [512, 640, 768, 896, 1024, 1280, 1280, 1536]
        input_size = input_sizes[self.compound_coef] if self.force_input_size is None else self.force_input_size
        ori_imgs, framed_imgs, framed_metas = preprocess_image(image, max_size=input_size)

        if self.use_cuda:
            x = torch.stack([torch.from_numpy(fi).cuda() for fi in framed_imgs], 0)
        else:
            x = torch.stack([torch.from_numpy(fi) for fi in framed_imgs], 0)

        x = x.to(torch.float32 if not self.use_float16 else torch.float16).permute(0, 3, 1, 2)
        # print("Preprocess time: %s"%(time.time()-t0))

        with torch.no_grad():
            t0 = time.time()
            features, regression, classification, anchors = self.model(x)
            # print("Inference time: %s"%(time.time()-t0))

            t0 = time.time()
            regressBoxes = BBoxTransform()
            clipBoxes = ClipBoxes()
            out = postprocess(x,
                            anchors, regression, classification,
                            regressBoxes, clipBoxes,
                            self.threshold, self.iou_threshold)
        out = invert_affine(framed_metas, out)
        # print("Postprocess time: %s"%(time.time()-t0))
        return out

    def display(self, preds, imgs, save_path, imshow=False, imwrite=True):
        for i in range(len(imgs)):
            if len(preds[i]['rois']) == 0:
                print("Fail to find box")
                continue
            for j in range(len(preds[i]['rois'])):
                (x1, y1, x2, y2) = preds[i]['rois'][j].astype(np.int)
                cv2.rectangle(imgs[i], (x1, y1), (x2, y2), (255, 255, 0), 2)
                obj = self.obj_list[preds[i]['class_ids'][j]]
                score = float(preds[i]['scores'][j])
                cv2.putText(imgs[i], '{}, {:.3f}'.format(obj, score),
                            (x1+5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 0, 255), 1)
            if imshow:
                cv2.imshow('img', imgs[i])
                cv2.waitKey(0)
            if imwrite:
                cv2.imwrite(save_path, imgs[i])

