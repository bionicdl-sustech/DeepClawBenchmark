import tensorflow as tf
import numpy as np
from fc_graspNet import fcmodel


class Predictor(object):
    def __init__(self, checkpoint_path, width=1280, height=720):
        self.w = width
        self.h = height
        self.images_batch = tf.placeholder(tf.float32, shape=[None, self.h, self.w, 3])
        self.init_model(checkpoint_path)

    def init_model(self, checkpoint_path):
        M = fcmodel()
        M.initialize_network(checkpoint_path)
        # M.initialize_network('/home/h/DeepClawBenchmark/Functions/SoftGripper/checkpoint/Network9-1000-60')
        self.logits = M.inference(self.images_batch)
        print(self.logits.get_shape())
        logits_r = tf.reshape(self.logits, [1, self.logits.get_shape()[1].value, self.logits.get_shape()[2].value,
                                            self.logits.get_shape()[3].value / 2, 2])
        self.y = tf.nn.softmax(logits_r)
        config = tf.ConfigProto()
        config.gpu_options.per_process_gpu_memory_fraction = 0.8
        self.sess = tf.Session(config=config)
        init_op = tf.group(tf.global_variables_initializer(), tf.local_variables_initializer())
        self.sess.run(init_op)
        self.uv_range = [7, 11]

    def predict(self, color_image):
        # img = image[:, :, ::-1]
        img = color_image.reshape(1, self.h, self.w, 3) - 164.0

        y_ = self.sess.run(self.y, feed_dict={self.images_batch: img})

        p_best = np.max(y_[0, self.uv_range[0]:15, self.uv_range[1]:28, :, 1], axis=2)
        location = np.where(p_best == p_best.max())
        best_u = 114 + location[1][0] * 32 + self.uv_range[1] * 32
        best_v = 114 + location[0][0] * 32 + self.uv_range[0] * 32
        theta = np.argmax(y_[0, self.uv_range[0]:15, self.uv_range[1]:24, :, 1], axis=2)
        best_theta = theta[location[0][0]][location[1][0]]
        best_prob = p_best.max()

        return y_,p_best,[best_u, best_v, 0.05, -3.14, 0,(best_theta + 0.5) * (1.57 / 9) - 1.57]
