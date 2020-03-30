#!/usr/bin/env python
# coding: utf-8

# You may need to restart your runtime prior to this, to let your installation take effect
# Some basic setup:
# Setup detectron2 logger
import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import some common libraries
import numpy as np
import cv2
import random
from google.colab.patches import cv2_imshow

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import DatasetCatalog, MetadataCatalog
import os
import json
from detectron2.structures import BoxMode
from detectron2.engine import DefaultTrainer, default_setup
from detectron2.config import get_cfg
# # Train on a custom dataset

DATA_PATH_ROOT = "/raid/wanfang/Documents/"

f = open("category_names.txt",'r')
for line in f.readlines():
    names = line.replace('\n','').replace('\'','').split(',')
    for i in range(1,len(names)):
        names[i] = names[i][1:]

f.close()

def get_names(img_dir):
    json_file = os.path.join(img_dir, "train.json")
    with open(json_file) as f:
        file = json.load(f)
    categories = file['categories']
    names = []
    for c in categories:
        names.append(c["name"])
    return names

def get_train_dicts(img_dir):
    json_file = os.path.join(img_dir,"train/train.json")
    with open(json_file) as f:
        file = json.load(f)
    # initialte dataset dict with filename and image_id
    images = file['images']
    dataset_dicts = []
    image_id_index = {}
    index = 0
    for d in images:
        record = {}
        filename = os.path.join(img_dir+"/train", d['file_name'])
        record["file_name"] = filename
        record["image_id"] = d['image_id']
        record["height"] = d['height']
        record["width"] = d['width']
        record["annotations"] = []
        dataset_dicts.append(record)
        image_id_index.update({d['image_id']:index})
        index += 1
    # update annotations
    imgs_anns = file['annotations']
    for d in imgs_anns:
        image_id = d['image_id']
        index = image_id_index[d['image_id']]
        bbox = {
            "bbox": d['bbox'],
            "bbox_mode": BoxMode.XYWH_ABS,
            "category_id": d['category_id']-1,
            "iscrowd": d['iscrowd']
        }
        dataset_dicts[index]["annotations"].append(bbox)
    return dataset_dicts

DatasetCatalog.clear()
d = "/raid/wanfang/Documents/complex_open_data"
DatasetCatalog.register("complex_trash_train", lambda d=d: get_train_dicts(d))
MetadataCatalog.get("complex_trash_train").set(thing_classes=names)
complex_metadata_train = MetadataCatalog.get("complex_trash_train")

dataset_dicts = get_train_dicts(d)
print("====================================There are %s complex training images."%len(dataset_dicts))
# There are 79717 training images.

# from PIL import Image, ImageDraw, ImageFont
# for d in random.sample(dataset_dicts, 5):
#     img = cv2.imread(d["file_name"])
#     visualizer = Visualizer(img[:, :, ::-1], metadata=single_metadata_train, scale=0.2)
#     vis = visualizer.draw_dataset_dict(d)
#     img_pil = Image.fromarray(vis.get_image())
#     draw = ImageDraw.Draw(img_pil)
#     font_text = ImageFont.truetype('./simsun.ttc', 20)
#     draw.text((100,10), names[d['annotations'][0]['category_id']-1], (0, 255, 0), font=font_text)
#     img_pil.show()

###################################################################################
# Step 2: Train!
cfg = get_cfg()
# cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml"))
cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_X_101_32x8d_FPN_3x.yaml"))
cfg.DATASETS.TRAIN = ("complex_trash_train",)
cfg.DATASETS.TEST = ()
cfg.DATALOADER.NUM_WORKERS = 2
cfg.MODEL.WEIGHTS = "output_X101_/model_0299999.pth"  # Let training initialize from model zoo
cfg.SOLVER.IMS_PER_BATCH = 2 # 16 is too big, 32 too big for v100
cfg.SOLVER.BASE_LR = 0.00025  # pick a good LR
cfg.SOLVER.MAX_ITER = 500000    # 300 iterations seems good enough for this toy dataset; you may need to train longer for a practical dataset
cfg.SOLVER.STEPS = [500000, 600000]
# cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, and good enough for this toy dataset (default: 512)
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 204  # only has one class (ballon)
cfg.OUTPUT_DIR = "output_X101_complex_"
cfg.INPUT.MIN_SIZE_TRAIN = (704, 1080)
cfg.INPUT.MAX_SIZE_TRAIN = (1920)
os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)

from torch.optim.lr_scheduler import StepLR
trainer = DefaultTrainer(cfg)
trainer.resume_or_load(resume=False)
trainer.scheduler = StepLR(trainer.optimizer, step_size=1, gamma=0.1)
# trainer.scheduler.milestones
trainer.train()
