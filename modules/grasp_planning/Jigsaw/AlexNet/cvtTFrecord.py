#! /usr/bin/env python
# coding=utf-8
#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#
#   Editor      : VIM
#   File name   : convert_tfrecord.py
#   Author      : YunYang1994
#   Created date: 2018-12-18 12:34:23
#   Description :
#
#================================================================
import sys
import argparse
import numpy as np
import tensorflow as tf
import glob, re
import cv2

#prepare data.txt: image_file_path boundingbox(top_left, bottom_right) class
numbers = re.compile(r'(\d+)')
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

def image2tfrecord(outputfolder='./data/train/TrainingData',tfrecord_path_prefix='./data/train/train'):
    dataset_txt = '/home/bionicdl-saber/Documents/git_project/finetune_alexnet_with_tensorflow/train_real.txt'
    dataset = {}
    with open(dataset_txt,'r') as f:
        for line in f.readlines():
            example = line.split(' ')
            print("HHHHHHHHHHHHHHHHH: ",example)
            image_path = example[0]
            label = example[1]
            dataset[image_path] = label

    image_paths = list(dataset.keys())
    images_num = len(image_paths)
    print(">> Processing %d images" %images_num)

    tfrecord_file = tfrecord_path_prefix+".tfrecords"
    with tf.python_io.TFRecordWriter(tfrecord_file) as record_writer:
        for i in range(images_num):
            image = tf.gfile.FastGFile(image_paths[i], 'rb').read()
            label = dataset[image_paths[i]]
            example = tf.train.Example(features = tf.train.Features(
                feature={
                    'image' :tf.train.Feature(bytes_list = tf.train.BytesList(value = [image])),
                    'label' :tf.train.Feature(bytes_list = tf.train.BytesList(value = [label])),
                }
            ))
            record_writer.write(example.SerializeToString())
        print(">> Saving %d images in %s" %(images_num, tfrecord_file))
