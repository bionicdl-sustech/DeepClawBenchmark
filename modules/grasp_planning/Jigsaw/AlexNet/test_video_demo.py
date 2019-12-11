import os, cv2, re
import numpy as np
import tensorflow as tf
from alexnet import AlexNet
import glob
from PIL import Image
import pyrealsense2 as rs

points = rs.points()
pipeline= rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)

while False:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    image = np.asanyarray(color_frame.get_data())
    ####################################################
    # process real image to remove texture
    # mask = cv2.inRange(image, np.array([0, 80, 0]),np.array([140, 255,65]))
    # kernel = np.ones((4,4),np.uint8)
    # mask = cv2.dilate(mask,kernel,iterations = 1)
    # background = np.full(image.shape, 178, dtype=np.uint8)
    # bk = cv2.bitwise_or(background, background, mask=mask)
    ###########################################
    crop_image = image[:, 280:280+720]
    image_resized = cv2.resize(crop_image, (227,227))
    img = cv2.resize(crop_image.astype(np.float32), (227,227))
    img = img.reshape((1,227,227,3))
    pred = sess.run(softmax, feed_dict={x: img})
    predicted_label = pred.argmax(axis=1)
    info = "type: %s" %(predicted_label[0])
    # print("%s is %s with probability: %s"%(all_images[i], predicted_label[0], pred[0,predicted_label[0]]))
    cv2.putText(image_resized, text=info, org=(10, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, color=(0, 0, 255), thickness=2)
    cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("result", image_resized)
    if cv2.waitKey(1) & 0xFF == ord('q'): break


class Grasp_predictor(object):
    """docstring for grasp_predictor."""
    def __init__(self):
        self.x = tf.placeholder(tf.float32, [1, 227, 227, 3])
        # Initialize model
        self.model = AlexNet(self.x, 1, 360, [])
        # Link variable to model output
        self.score = self.model.fc8
        self.softmax = tf.nn.softmax(self.score)
        # create saver instance
        self.saver = tf.train.Saver()
        self.sess = tf.Session()
        self.saver.restore(self.sess, tf.train.latest_checkpoint('./checkpoints'))
    def predict_rotation(self, image):
        image = cv2.resize(image,(227,227))
        image = image.astype(np.float32)
        image.resize([1,227,227,3])
        pred = self.sess.run(self.softmax, feed_dict={self.x: image})
        self.predicted_label = pred.argmax(axis=1)
        return self.predicted_label
    def predict_rotation_2(self, image):
        image = cv2.resize(image,(227,227))
        I = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
        images = []
        for i in range(360):
            I_i = I.rotate(-i,expand=0,fillcolor=(182,182,180))
            image_rotate_rgb = np.array(I_i)
            image_rotate_bgr = image_rotate_rgb[:, :, ::-1]
            images.append(image_rotate_bgr.astype(np.float32))
        images = np.array(images)
        pred = self.sess.run(self.softmax, feed_dict={self.x: images})
        self.predicted_labels = pred.argmax(axis=1)
        locs_0 = np.where(self.predicted_labels[:] == 0)[0]
        if locs_0.size != 0:
            loc = np.median(locs_0)
            angle = 360 - loc
            return angle
        # if 2 just to 1 directly without 0
        locs = np.where(self.predicted_labels[:-1] - self.predicted_labels[1:] == 1)[0]
        if locs.size == 0:
            angle = 0
        else:
            angle = -999
            for loc in locs:
                # exclude situations with 1 1 2 1 1, 2 2 1 2 2
                if self.predicted_labels[loc] - self.predicted_labels[(loc+2)%359] == 1 and self.predicted_labels[loc] - self.predicted_labels[(loc-1)%359]==0:
                    angle = 360 - (loc + 1)
        return angle
    def close():
        self.sess.close()

predictor = Grasp_predictor()
while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    image = np.asanyarray(color_frame.get_data())
    crop_image = image[:, 280:280+720]
    angle = predictor.predict_rotation(crop_image)
    info = "Angle: %s"%(angle[0])
    cv2.putText(crop_image, text=info, org=(10, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, color=(0, 0, 255), thickness=2)
    cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("result", crop_image)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
