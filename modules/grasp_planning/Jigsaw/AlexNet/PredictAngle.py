import os, cv2, re
import numpy as np
import tensorflow as tf
from alexnet import AlexNet
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
class Grasp_predictor(object):
    """docstring for grasp_predictor."""
    def __init__(self):
        # gpu_options = tf.GPUOptions(allow_growth=True,per_process_gpu_memory_fraction=0.3)
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.3)
        __config = tf.ConfigProto(log_device_placement=True, gpu_options=gpu_options)
        __g1 = tf.Graph()

        with __g1.as_default():
            self.x = tf.placeholder(tf.float32, [1, 227, 227, 3])
            # Initialize model
            self.model = AlexNet(self.x, 1, 360, [])
            # Link variable to model output
            self.score = self.model.fc8
            self.softmax = tf.nn.softmax(self.score)
            # create saver instance
            self.saver = tf.train.Saver()
            saver_def = self.saver.as_saver_def()


        self.sess1 = tf.Session(graph=__g1,config=__config)
        with __g1.as_default():
            # self.saver.restore(self.sess, tf.train.latest_checkpoint('./checkpoints'))
            ckpt_filename = 'model_epoch100.ckpt'
            __currentPath = os.path.dirname(os.path.abspath(__file__))
            self.saver.restore(self.sess1, __currentPath +'/checkpoints/'+ ckpt_filename)

    def predict_rotation(self, image):
        image = cv2.resize(image,(227,227))
        image = image.astype(np.float32)
        image.resize([1,227,227,3])
        pred = self.sess1.run(self.softmax, feed_dict={self.x: image})
        self.predicted_label = pred.argmax(axis=1)
        # self.close()
        return self.predicted_label
    def close():
        self.sess1.close()
if __name__ == '__main__':
        predictor = Grasp_predictor()
        img = cv2.imread(_root_path + '/data/test_data/ori.jpg')
        angle = predictor.predict_rotation(img)
        print('angle: ',angle[0])
