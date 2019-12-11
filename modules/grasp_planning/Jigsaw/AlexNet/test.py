import os, cv2, re
import numpy as np
import tensorflow as tf
from alexnet import AlexNet
import glob

numbers = re.compile(r'(\d+)')
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

all_images = sorted(glob.glob('./val_data_real/*.png'), key=numericalSort)
imgs = []
for f in all_images:
    image = cv2.imread(f)
    imgs.append(image)

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

# all_images = sorted(glob.glob('./jigsaw_images_2/*.jpg'), key=numericalSort)
# imgs = []
# for f in all_images:
#     image = cv2.imread(f)
#     mask = cv2.inRange(image, np.array([0, 80, 0]),np.array([140, 255,65]))
#     kernel = np.ones((4,4),np.uint8)
#     mask = cv2.dilate(mask,kernel,iterations = 1)
#     # mask = cv2.bitwise_not(mask)
#     background = np.full(image.shape, 178, dtype=np.uint8)
#     bk = cv2.bitwise_or(background, background, mask=mask)
#     imgs.append(bk)
#     cv2.imshow("Image",bk)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

if __name__ == '__main__':
    predictor = Grasp_predictor()
    img = cv2.imread('/home/bionicdl-saber/Documents/git_project/finetune_alexnet_with_tensorflow/val_data_real/image_cat_2_0.0997253101362.png')
    angle = predictor.predict_rotation(img)
    print('angle: ',angle[0])


# predictor = Grasp_predictor()
# for i in range(len(imgs)):
#     # i=26
#     image = imgs[i]
#     angle = predictor.predict_rotation(image)
#     info = "%s. Angle: %s, T: %s"%(i, angle[0], all_images[i][28:33].split('.')[0])
#     cp = image.copy()
#     cv2.putText(cp, text=info, org=(10, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
#                     fontScale=0.7, color=(0, 0, 255), thickness=2)
#     cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
#     cv2.imshow("result", cp)
#     # cv2.waitKey(0)
#     # cv2.destroyAllWindows()
#     if cv2.waitKey(1000) & 0xFF == ord('q'): break
#
# cv2.destroyAllWindows()
