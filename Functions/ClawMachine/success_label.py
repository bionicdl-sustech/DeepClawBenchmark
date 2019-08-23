import sys
import time
import os

root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.append(root_path)

from Driver.Camera.RealsenseController import RealsenseController
import DetectForeground as df
def success_label(img1,img2):


    img1 = img1[0:120,450:670]



    img2 = img2[0:120,450:670]

    compare = df.Segment()
    rect = compare.DiffGround(img1,img2)
    if len(rect) > 0 :
        success_label = 1
        print('success_label:'+str(success_label))
        return success_label
    else :
        success_label = 0
        print('success_label:'+str(success_label))

        return success_label

    # print(rect[0])
    #
    # imag = cv2.rectangle(imagegray,(rect[0][0],rect[0][1]),(rect[0][0]+rect[0][2],rect[0][1]+rect[0][3]),(0,255,0),3)
    # cv2.imshow('img',imag)
    # cv2.waitKey()

    # print(rect)
if __name__=="__main__":
    camera = RealsenseController()
    img1,depth_image,infrared_L,infrared_R = camera.getImage()
    time.sleep(3)
    img1,depth_image,infrared_L,infrared_R = camera.getImage()
    time.sleep(1)
    img2,depth_image,infrared_L,infrared_R = camera.getImage()
    a = success_label(img1,img2)
    print(a)
