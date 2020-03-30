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
from detectron2.data import MetadataCatalog
import os
import numpy as np
import json
from detectron2.structures import BoxMode

from detectron2.engine import DefaultTrainer, default_setup
from detectron2.config import get_cfg
# # Train on a custom dataset

DATA_PATH_ROOT = "/raid/wanfang/Documents/"

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
    json_file = os.path.join(img_dir, "train.json")
    with open(json_file) as f:
        file = json.load(f)
    # initialte dataset dict with filename and image_id
    images = file['images']
    dataset_dicts = []
    image_id_index = {}
    index = 0
    for d in images:
        record = {}
        filename = os.path.join(img_dir, d['file_name'])
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
            "category_id": d['category_id'],
            "iscrowd": d['iscrowd']
        }
        dataset_dicts[index]["annotations"].append(bbox)
    return dataset_dicts


def get_val_dicts(img_dir):
    json_file = os.path.join(img_dir, "val_open.json")
    with open(json_file) as f:
        file = json.load(f)
    # initialte dataset dict with filename and image_id
    images = file['images']
    dataset_dicts = []
    image_id_index = {}
    index = 0
    for d in images:
        record = {}
        filename = os.path.join(img_dir, d['file_name'])
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
            "category_id": d['category_id'],
            "iscrowd": d['iscrowd']
        }
        dataset_dicts[index]["annotations"].append(bbox)
    return dataset_dicts


def get_single_dicts(t):
    if t=="train":
        json_file = DATA_PATH_ROOT+"train/train.json"
        with open(json_file) as f:
            file = json.load(f)
        # initialte dataset dict with filename and image_id
        images = file['images']
        dataset_dicts = []
        image_id_index = {}
        index = 0
        for d in images:
            filename = os.path.join(DATA_PATH_ROOT+t, d['file_name'])
            if not os.path.exists(filename):
            # if cv2.imread(filename) is None:
                print("Found not exiting file %s")
                continue
            record = {}
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
            if image_id in image_id_index:
                index = image_id_index[d['image_id']]
                bbox = {
                    "bbox": d['bbox'],
                    "bbox_mode": BoxMode.XYWH_ABS,
                    "category_id": d['category_id'],
                    "iscrowd": d['iscrowd']
                }
                dataset_dicts[index]["annotations"].append(bbox)
    elif t=="val":
        json_file = DATA_PATH_ROOT+"val/val_open.json"
        with open(json_file) as f:
            file = json.load(f)
        # initialte dataset dict with filename and image_id
        images = file['images']
        dataset_dicts = []
        image_id_index = {}
        index = 0
        for d in images:
            record = {}
            filename = os.path.join(DATA_PATH_ROOT+t, d['file_name'])
            record["file_name"] = filename
            record["image_id"] = d['image_id']
            record["height"] = d['height']
            record["width"] = d['width']
            record["annotations"] = []
            dataset_dicts.append(record)
            image_id_index.update({d['image_id']:index})
            index += 1
    return dataset_dicts


names = get_names(DATA_PATH_ROOT+'train')

from detectron2.data import DatasetCatalog, MetadataCatalog

DatasetCatalog.clear()
d = "train"
DatasetCatalog.register("single_trash_train", lambda d=d: get_single_dicts(d))
MetadataCatalog.get("single_trash_train").set(thing_classes=names)
single_metadata_train = MetadataCatalog.get("single_trash_train")

# d = "val"
# DatasetCatalog.register("single_trash_val", lambda d=d: get_single_dicts(d))
# MetadataCatalog.get("single_trash_val").set(thing_classes=['0','1','nut'])
# single_metadata_val = MetadataCatalog.get("single_trash_val")


# To verify the data loading is correct, let's visualize the annotations of randomly selected samples in the training set:

dataset_dicts = get_single_dicts("train")
print("====================================There are %s training images."%len(dataset_dicts))


cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml"))
cfg.DATASETS.TRAIN = ("single_trash_train",)
cfg.DATASETS.TEST = ()
cfg.DATALOADER.NUM_WORKERS = 2
cfg.MODEL.WEIGHTS = "model_final_280758.pkl"  # Let training initialize from model zoo
cfg.SOLVER.IMS_PER_BATCH = 2 # 16 is too big, 32 too big for v100
cfg.SOLVER.BASE_LR = 0.00025  # pick a good LR
cfg.SOLVER.MAX_ITER = 10000    # 300 iterations seems good enough for this toy dataset; you may need to train longer for a practical dataset
cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, and good enough for this toy dataset (default: 512)
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 204  # only has one class (ballon)

cfg.OUTPUT_DIR = "output_2"
# ## Inference & evaluation using the trained model
# Now, let's run inference with the trained model on the balloon validation dataset. First, let's create a predictor using the model we just trained:

cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
cfg.DATASETS.TEST = ("single_trash_train", )
predictor = DefaultPredictor(cfg)


# Then, we randomly select several samples to visualize the prediction results.

# dataset_dicts = get_single_dicts("val")
i = 0
from PIL import Image, ImageDraw, ImageFont
for d in random.sample(dataset_dicts, 10):
    im = cv2.imread(d["file_name"])
    if im is None:
      continue
    outputs = predictor(im)
    v = Visualizer(im[:, :, ::-1],
                   metadata=single_metadata_train,
                   scale=0.3,
    )
    v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    l = outputs["instances"].get("pred_classes").cpu().numpy()
    img_pil = Image.fromarray(v.get_image())
    draw = ImageDraw.Draw(img_pil)
    font_text = ImageFont.truetype('./simsun.ttc', 20)
    if len(l) == 0:
        print("Fail to find anything")
        draw.text((10,10), "Fail", (255, 0, 0), font=font_text)
        draw.text((100,10), names[d['annotations'][0]['category_id']], (0, 255, 0), font=font_text)
        img_pil.save("images_test/Test_%s.png"%i)
    else:
        print("Find: ",names[l[0]])
        draw.text((10,10), names[l[0]], (255, 0, 0), font=font_text)
        draw.text((100,10), names[d['annotations'][0]['category_id']], (0, 255, 0), font=font_text)
        img_pil.save("images_test/Test_%s.png"%i)
    i=i+1


# We can also evaluate its performance using AP metric implemented in COCO API.
# This gives an AP of ~70%. Not bad!

# from detectron2.evaluation import COCOEvaluator, inference_on_dataset
# from detectron2.data import build_detection_test_loader
# evaluator = COCOEvaluator("single_trash_train", cfg, False, output_dir="./output_2/")
# val_loader = build_detection_test_loader(cfg, "single_trash_train")
# inference_on_dataset(trainer.model, val_loader, evaluator)
# another equivalent way is to use trainer.test

#You can remove "~/.torch/fvcore_cache" and try again.
