# import tf_utils
#
# import time
# import os
#
# from graspNet import model as grasp_net
#
# import tensorflow as tf
# import tensorflow.contrib.framework as tcf
#
# # 40, 10min/epoch;
# batch_size = 100
# num_epochs = 15
# #starter_learning_rate = 0.05
# use_gpu_fraction = 1
#
# max_learning_rate = 0.001
# min_learning_rate = 0.0001
# decay_speed = 1000
#
# checkpoint_path = './checkpoint'
# summary_path = './summary'
# data_path = '/home/bionicdl/git-projects-py2/as_DeepClaw/training/croppedImage_tfrecord'
# ckpt_file = tf.train.latest_checkpoint(checkpoint_path)
# reader = tf.train.NewCheckpointReader(ckpt_file)
#
# # Create empty weight object.
# weights = {}
#
# # Read/generate weight/bias variable names.
# ckpt_vars = tcf.list_variables(ckpt_file)
# full_var_names = []
# short_names = []
# for variable, shape in ckpt_vars:
#     full_var_names.append(variable)
#     short_names.append(variable.split("/")[-1])
#
# # Load variables.
# for full_var_name, short_name in zip(full_var_names, short_names):
#     weights[short_name] = tf.Variable(reader.get_tensor(full_var_name), name=full_var_name)
#
# _build_conv_layer
# _build_fc_layer
#
# checkpoint = tf.train.latest_checkpoint(checkpoint_path)

# initialize prediction network for each patch
from fc_graspNet import fcmodel
# from PIL import Image, ImageDraw
import tensorflow as tf
import numpy as np
import cv2

class Predictor(object):
    def __init__(self):
        self.w = 1280
        self.h = 720
        self.images_batch = tf.placeholder(tf.float32, shape=[None, self.h, self.w, 3])
        M = fcmodel()
        # M.initialize_network('/home/h/DeepClawBenchmark/Functions/SoftGripper/checkpoint/Network1-1000-100')
        M.initialize_network('/home/h/DeepClawBenchmark/Functions/SoftGripper/checkpoint/Network9-1000-60')
        self.logits = M.inference(self.images_batch)
        logits_r = tf.reshape(self.logits, [1, self.logits.get_shape()[1].value, self.logits.get_shape()[2].value,
                                       self.logits.get_shape()[3].value/2,2])
        self.y = tf.nn.softmax(logits_r)
        config = tf.ConfigProto()
        config.gpu_options.per_process_gpu_memory_fraction = 0.8
        self.sess = tf.Session(config = config)
        init_op = tf.group(tf.global_variables_initializer(), tf.local_variables_initializer())
        self.sess.run(init_op)
        self.uv_range = [0, 11]

    def locate_object(self, image):
        # img = image[:, :, ::-1]
        img = image.reshape(1, self.h, self.w, 3) - 164.0
        # logits_ = self.sess.run(self.logits, feed_dict={self.images_batch:img})
        y_ = self.sess.run(self.y, feed_dict={self.images_batch: img})
        p_best = np.max(y_[0, self.uv_range[0]:15, self.uv_range[1]:24, :, 1], axis=2)
        location = np.where(p_best == p_best.max())
        best_u = 114 + location[1][0] * 32 + self.uv_range[1] * 32
        best_v = 114 + location[0][0] * 32 + self.uv_range[0] * 32
        theta = np.argmax(y_[0, self.uv_range[0]:15, self.uv_range[1]:24, :, 1], axis=2)[location[0][0]][location[1][0]]
        best_prob = p_best.max()
        for i in range(p_best.shape[0]):
            for j in range(p_best.shape[1]):
                u = 114 + j * 32 + self.uv_range[1] * 32
                v = 114 + i * 32 + self.uv_range[0] * 32
                r = p_best[i, j] * 16
                cv2.circle(image, (u, v), int(r), (0, 0, 255), -1)
        cv2.circle(image, (best_u, best_v), 20, (0, 255, 0), -1)
        return image, [best_u, best_v], best_prob, (0.5 + theta) * (3.14 / 9) - 1.57


# I = Image.open(path)
# draw = ImageDraw.Draw(I, 'RGBA')
# for i in range(p_best.shape[0]):
#     for j in range(p_best.shape[1]):
#         u = j*35
#         v = i*40
#         r = p_best[i,j] * 17
#         draw.ellipse((u-r, v-r, u+r, v+r), (0, 0, 255, 125))
#
# I.save("test_p_.png")
