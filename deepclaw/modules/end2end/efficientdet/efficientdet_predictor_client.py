import numpy as np
import os, time
import sys
import cv2
from deepclaw.utils.Client import Client


class efficientdet(object):
    def __init__(self, compound_coef = 0, weight_path = None, num_classes = 204):
        if num_classes == 4:
            self.obj_list = ['glass', 'paper', 'metal', 'plastic']
        else:
            self.obj_list = ['seed shell', 'walnut', 'peanut shell', 'edamame shell', 'watermelon seed', 'date pit', 'plum pit', 'apple peel', 'persimmon peel', 'watermelon rind', 'banana peel', 'shaddock peel', 'litchi shell', 'mango peel', 'apple core', 'dried fruit', 'orange peel', 'biscuit', 'bread', 'candy', 'pet food', 'air-dried food', 'candied fruit', 'jerky', 'brewed beverage powder', 'cheese', 'can', 'cake', 'potato chips', 'leaves', 'weeds', 'green plant', 'fresh flowers', 'beans', 'animal organs', 'mung bean rice', 'cereals and processed products', 'shelled without shell', 'shrimp', 'pasta', 'meat', 'whole grains', 'rib', 'chicken', 'chicken bones', 'snail', 'duck', 'fish', 'vegetable root', 'cauliflower', 'mushrooms', 'fish scale', 'seasoning', 'tea residue', 'coffee grounds', 'zongzi', 'animal hoof', 'crayfish', 'crab shell', 'sauce', 'fish bones', 'eggshell', 'Chinese herbal medicine', 'Chinese medicine residue', 'mirror', 'glass product', 'window glass', 'broken glass', 'cosmetic glass bottle', 'glass jars for food and daily necessities', 'vacuum cup', 'glass', 'journal', 'newspaper', 'food packaging box', 'shoe box', 'tetra pack', 'leaflets', 'printer paper', 'paper shopping bag', 'calendar', 'paper courier bags', 'envelope', 'cigarette case', 'cans', 'metallica', 'magnet', 'aluminium product', 'metal bottles', 'metal tools', 'tin can', 'spoon', 'kitchen knife', 'fork', 'pot', 'metal chopsticks', 'USB cable', 'plastic toy', 'mineral water bottle', 'plastic foam', 'plastic package', 'rigid plastic', 'disposable plastic dinnerware', 'electric wire', 'plastic hanger', 'melamine tableware', 'acrylic board', 'PVC tube', 'socket', 'cosmetic plastic bottle', 'basketball', 'football', 'KT board', 'food plastic box', 'cooking oil containers', 'plastic cup', 'plastic bottle', 'disposable lunch box', 'discarded clothes', 'shoe', 'cloth waste', 'school bag', 'bedding', 'quilt', 'silk handkerchief', 'pillow', 'plush toy', 'belt', 'circuit board', 'power bank', 'woodware', 'flash disk', 'lamp bulb', 'energy saving lamp', 'diode', 'button cell', 'cell phone battery', 'nickel--cadmiun battery', 'lithium battery', 'accumulator', 'film', 'photograph', 'nail', 'X-ray film', 'pesticide bottle', 'insecticides and cans', 'candle', 'ink cartridge', 'hair dye shell', 'disinfectant bottle', 'paint bucket', 'drug packaging', 'medicine bottle', 'waste needle tube', 'infusion tube', 'oral liquid bottle', 'eye drops bottle', 'mercurial thermometer', 'mercurial sphygmomanometer', 'capsule', 'tablet', 'solid insecticide', 'sugarcane skin', 'nutshell', 'plasticine', 'hair', 'swab', 'band-aid', 'lipstick', 'pen', 'tissue', 'tape', 'wet tissue', 'water color pen', 'lighter', 'bubble shatter-proof warp', 'durian shell', 'eye black', 'eye shadow', 'hamster sand bath', 'big bone rods', 'old towel', 'bamboo ware', 'chalk', 'disposable mask', 'disposable glove', 'liquid foundation', 'dust', 'nylon products', 'diaper', 'umbrella', 'tape product', 'toothpaste', 'the dog diapers', 'coconut shell', 'powder puff', 'broken dishes', 'ceramic', 'toilet paper', 'cigarette end', 'false eyelashes', 'cat litter', 'toothbrush', 'corn cob']

        data = {'type': 'instruction',
                'data': ['from deepclaw.modules.end2end.efficientdet.efficientdet_predictor import efficientdet',
                         [(compound_coef, weight_path, num_classes)],
                         'efficientdet']}
        self.client = Client(host_ip='192.168.1.102', port=2020)
        self.client.start()
        self.client.send(data)
        self.client.close()

    def run(self, image_np):
        self.client.start()
        # TODO: a bug here, color_image should be ndarray type, but json cannot handle this type.
        data = {'type': 'image',
                'data': image_np.tolist()}
        t0 = time.time()
        self.client.send(data)
        print("\nSend time cost: %s"%(time.time()-t0))

        t0 = time.time()
        feedback = self.client.recv()
        dt = time.time() - t0
        print("Receive time cost: %s"%dt)
        output_dict = feedback['feedback']
        output_dict[0]['rois'] = np.array(output_dict[0]['rois'])
        output_dict[0]['class_ids'] = np.array(output_dict[0]['class_ids'])
        output_dict[0]['scores'] = np.array(output_dict[0]['scores'])
        return output_dict

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