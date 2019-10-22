import time
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
        M.initialize_network('/home/h/DeepClawBenchmark/Functions/SoftGripper/checkpoint/Network1-1000-100')
        # M.initialize_network('/home/h/DeepClawBenchmark/Functions/SoftGripper/checkpoint/Network9-1000-60')
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
        start = time.time()
        y_ = self.sess.run(self.y, feed_dict={self.images_batch: img})
        print(time.time() - start)
        p_best = np.max(y_[0, self.uv_range[0]:15, self.uv_range[1]:24, :, 1], axis=2)
        location = np.where(p_best == p_best.max())
        best_u = 114 + location[1][0] * 32 + self.uv_range[1] * 32
        best_v = 114 + location[0][0] * 32 + self.uv_range[0] * 32
        theta = np.argmax(y_[0, self.uv_range[0]:15, self.uv_range[1]:24, :, 1], axis=2)
        best_theta = theta[location[0][0]][location[1][0]]
        best_prob = p_best.max()
        for i in range(p_best.shape[0]):
            for j in range(p_best.shape[1]):
                u = 114 + j * 32 + self.uv_range[1] * 32
                v = 114 + i * 32 + self.uv_range[0] * 32
                r = p_best[i, j] * 16
                cv2.circle(image, (u, v), int(r), (0, 0, 255), -1)
                # cv2.ellipse(image, (u, v), (int(r), int(r/3)), -((0.5 + theta[i][j]) * (3.14 / 9) - 1.57) * 180 / 3.14, 0, 360, (0, 0, 255), -1)
        # cv2.circle(image, (best_u, best_v), 20, (0, 255, 0), -1)
        # cv2.ellipse(image, (best_u, best_v), (20, int(20 / 3)), -((0.5 + best_theta) * (3.14 / 9) - 1.57) * 180 / 3.14, 0, 360, (0, 255, 0), -1)
        return image, [best_u, best_v], best_prob, (0.5 + best_theta) * (3.14 / 9) - 1.57
