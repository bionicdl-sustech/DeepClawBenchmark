#predict the pose for grasp
import numpy as np
import os
__currentPath = os.path.dirname(os.path.abspath(__file__))
import sys
sys.path.append(__currentPath)
root_path = os.path.dirname(os.path.dirname(os.path.dirname(__currentPath)))
sys.path.append(root_path)
import cv2
import time

sys.path.append(root_path + '/modules/end2end/SSD/SSD-Tensorflow')
from nets import ssd_vgg_300, ssd_common, np_methods
from preprocessing import ssd_vgg_preprocessing
from notebooks import visualization

import tensorflow as tf
slim = tf.contrib.slim

class seg_rec(object):
    def __init__(self,workspace = [0,0,1280,720]):
        #color space
        self.colors_tableau = [(255, 255, 255), (31, 119, 180), (174, 199, 232), (255, 127, 14), (255, 187, 120),
                  (44, 160, 44), (152, 223, 138), (214, 39, 40), (255, 152, 150),
                  (148, 103, 189), (197, 176, 213), (140, 86, 75), (196, 156, 148),
                  (227, 119, 194), (247, 182, 210), (127, 127, 127), (199, 199, 199),
                  (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]
        #workspace
        self.ws = workspace #[xmin,ymin,xmax,ymax]
        #SSDNet
        # TensorFlow session: grow memory when needed. TF, DO NOT USE ALL MY GPU MEMORY!!!
        gpu_options = tf.GPUOptions(allow_growth=True,per_process_gpu_memory_fraction=0.5)
        config = tf.ConfigProto(log_device_placement=False, gpu_options=gpu_options)
        __g2 = tf.Graph()
        self.isess = tf.InteractiveSession(graph=__g2,config=config)
        # Input placeholder.
        self.net_shape = (300, 300)
        data_format = 'NHWC'
        self.img_input = tf.placeholder(tf.uint8, shape=(None, None, 3))
        # Evaluation pre-processing: resize to SSD net shape.
        image_pre, labels_pre, bboxes_pre, self.bbox_img = ssd_vgg_preprocessing.preprocess_for_eval(
            self.img_input, None, None, self.net_shape, data_format, resize=ssd_vgg_preprocessing.Resize.WARP_RESIZE)
        self.image_4d = tf.expand_dims(image_pre, 0)
        # Define the SSD model.
        reuse = True if 'ssd_net' in locals() else None
        self.ssd_net = ssd_vgg_300.SSDNet()
        with slim.arg_scope(self.ssd_net.arg_scope(data_format=data_format)):
            self.predictions, self.localisations, _, _ = self.ssd_net.net(self.image_4d, is_training=False, reuse=reuse)

    def setCheckPoint(self,ckpt_filePath = root_path + '/modules/end2end/SSD/SSD-Tensorflow/checkpoints/model.ckpt-454522'):
        self.ckpt_filename = ckpt_filePath

    def display(self,color_image):
        # Restore SSD model.
        self.color_image = color_image
        ckpt_filename = self.ckpt_filename
        self.isess.run(tf.global_variables_initializer())
        saver = tf.train.Saver()
        saver.restore(self.isess, ckpt_filename)
        # SSD default anchor boxes.
        self.ssd_anchors = self.ssd_net.anchors(self.net_shape)
        #class and position
        rclasses, rscores, rbboxes =  self.process_image(color_image)
        self.rclasses = []
        self.rscores = []
        self.rbboxes = []
        rbboxes_return = []
        for j in range(len(rclasses)):
            ymin = int(rbboxes[j][0]*color_image.shape[0])
            xmin = int(rbboxes[j][1]*color_image.shape[1])
            ymax = int(rbboxes[j][2]*color_image.shape[0])
            xmax = int(rbboxes[j][3]*color_image.shape[1])
            if (xmax < self.ws[0] or xmin > self.ws[2] or ymax <self.ws[1] or ymin > self.ws[3]):
                continue
            else:
                self.rclasses.append(rclasses[j])
                self.rscores.append(rscores[j])
                rbboxes_return.append([xmin,ymin,xmax,ymax])
                self.rbboxes.append(rbboxes[j])

        self.rclasses = np.array(self.rclasses)
        self.rscores = np.array(self.rscores)
        self.rbboxes = np.array(self.rbboxes)
        rbboxes_return = np.array(rbboxes_return)
        return self.rclasses, self.rscores, rbboxes_return

    def process_image(self,img, select_threshold=0.5, nms_threshold=.45, net_shape=(300, 300)):
        # Run SSD network.
        rimg, rpredictions, rlocalisations, rbbox_img = self.isess.run([self.image_4d, self.predictions, self.localisations, self.bbox_img],
                                                                  feed_dict={self.img_input: img})

        # Get classes and bboxes from the net outputs.
        rclasses, rscores, rbboxes = np_methods.ssd_bboxes_select(
                rpredictions, rlocalisations, self.ssd_anchors,
                select_threshold=select_threshold, img_shape=net_shape, num_classes=21, decode=True)

        rbboxes = np_methods.bboxes_clip(rbbox_img, rbboxes)
        rclasses, rscores, rbboxes = np_methods.bboxes_sort(rclasses, rscores, rbboxes, top_k=400)
        rclasses, rscores, rbboxes = np_methods.bboxes_nms(rclasses, rscores, rbboxes, nms_threshold=nms_threshold)
        # Resize bboxes to original image shape. Note: useless for Resize.WARP!
        rbboxes = np_methods.bboxes_resize(rbbox_img, rbboxes)
        return rclasses, rscores, rbboxes

    def show(self):
        color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        visualization.plt_bboxes(color_image, self.rclasses, self.rscores, self.rbboxes)

    def saveImage(self,save_Path='1.png'):
        color_image = self.color_image.copy()
        shape = self.color_image.shape
        for i in range(self.rbboxes.shape[0]):
            bbox = self.rbboxes[i]
            color = self.colors_tableau[i]
            # Draw bounding box
            p1 = (int(bbox[0] * shape[0]), int(bbox[1] * shape[1]))
            p2 = (int(bbox[2] * shape[0]), int(bbox[3] * shape[1]))
            cv2.rectangle(color_image, p1[::-1], p2[::-1], color, 2)
            # Draw text
            s = '%s/%.3f' % (self.rclasses[i], self.rscores[i])
            p1 = (p1[0]-5, p1[1])
            cv2.putText(color_image, s, p1[::-1], cv2.FONT_HERSHEY_DUPLEX, 0.4, color, 1)

        cv2.imwrite(save_Path,color_image)


if __name__ == '__main__':

    color_image=cv2.imread('../../../data/test_data/ori.jpg',1)

    pa = seg_rec([0,0,1280,720])
    pa.setCheckPoint(root_path + '/modules/end2end/SSD/SSD-Tensorflow/checkpoints/model.ckpt-454522')

    rclasses, rscores, rbboxes = pa.display(color_image)
    print(rscores)
    pa.show()
    # pa.saveImage('2.jpg')
    # cv2.imwrite('3.jpg',color_image)
