import os, cv2, re
import numpy as np
from alexnet import AlexNet
import glob
from PIL import Image

numbers = re.compile(r'(\d+)')
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

#############################################################################
# take the jigsaw part from the background and warpPerspective
# mask = cv2.inRange(image, np.array([0, 80, 0]),np.array([170, 255,65]))
# kernel = np.ones((4,4),np.uint8)
# mask = cv2.dilate(mask,kernel,iterations = 1)
# mask_inverse = cv2.bitwise_not(mask)
# fg = cv2.bitwise_or(image, image, mask=mask_inverse)

# cv2.imshow("Image",image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
#
# im, cnts, hierarchy = cv2.findContours(mask_inverse.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# gamebox = None
# for c in cnts:
#   rect = cv2.minAreaRect(c)
#   box = cv2.boxPoints(rect)
#   if np.sum(np.abs(box[0]-box[1]) + np.abs(box[1]-box[2]))>50:
#     gamebox = np.int0(box)
#
# br = gamebox[np.argmax(np.sum(gamebox,axis=1))]
# bl = gamebox[ (np.argmax(np.sum(gamebox,axis=1))+1)%4 ]
# tl = gamebox[ (np.argmax(np.sum(gamebox,axis=1))+2)%4 ]
# tr = gamebox[ (np.argmax(np.sum(gamebox,axis=1))+3)%4 ]
# maxWidth = 400
# maxHeight = 400
# src = np.float32([tl, tr, br, bl])
# dst = np.array([
#   [0, 0],
#   [maxWidth - 1, 0],
#   [maxWidth - 1, maxHeight - 1],
#   [0, maxHeight - 1]], dtype = "float32")
# M = cv2.getPerspectiveTransform(src, dst)
# jigsaw2 = cv2.warpPerspective(fg, M, (maxWidth, maxHeight)) #use bk if nontexture

################################################################################
# generate training data
def generate_data():
    train_data_path = "./train_data_real"
    if not os.path.isdir(train_data_path):
        os.mkdir(train_data_path)

    val_data_path = "./val_data_real"
    if not os.path.isdir(val_data_path):
        os.mkdir(val_data_path)

    all_images = sorted(glob.glob('./jigsaw_images/*.png'), key=numericalSort)
    # train_file = open("train_real.txt", "a")
    val_file = open("val_real.txt", "a")
    images2 = all_images[1::4]
    for j in range(6):
        image = cv2.imread(images2[j])
        image = cv2.resize(image,(277,277))
        I = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
        I_0 = I.rotate(-1,expand=0,fillcolor=(33,128,37))
        I_0.show()
        model_name = images2[j][16:-4]
        #
        for i in range(1800):
            It = I_0.rotate(-i,expand=1,fillcolor=(33,128,37))
            du = np.random.randint(-20, 20) #left/right (i.e. 5/-5)
            dv = np.random.uniform(-20, 20) #up/down (i.e. 5/-5)
            It = It.transform(It.size, Image.AFFINE, (1, 0, du, 0, 1, dv),fillcolor=(33,128,37))
            It.save('./train_data_real/image_%s_%s.png'%(model_name,i))
            if 5<i%360<180:
                label = 1
            elif 180<i%360<355:
                label = 2
            else:
                label = 0
            train_file.write('./train_data_real/image_%s_%s.png %s\n'%(model_name,i,i%360))
        for i in range(360):
            a = np.random.uniform(0, 360)
            It = I_0.rotate(-a,expand=1,fillcolor=(33,128,37))
            It = It.transform(It.size, Image.AFFINE, (1, 0, np.random.randint(-20, 20), 0, 1, np.random.randint(-20, 20)),fillcolor=(33,128,37))
            It.save('./val_data_real/image_%s_%s.png'%(model_name,a))
            if 5<a%360<180:
                label = 1
            elif 180<a%360<355:
                label = 2
            else:
                label = 0
            val_file.write('./val_data_real/image_%s_%s.png %s\n'%(model_name,a,int(a%360)))

    # train_file.close()
    val_file.close()
if __name__ == "__main__":
    generate_data()
#############################################################
# Inference
