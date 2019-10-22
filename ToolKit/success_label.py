import DetectForeground as df
import cv2

def success_label(imgray,color_image):
    imgray = imgray[480:680, 170:350]
    # cv2.imwrite("/home/h/DeepClawBenchmark/Data/before.jpg", imgray)
    # imgray = imgray[820:1020, 300:450]
    # cv2.imshow('first_image', imgray)
    # cv2.waitKey()

    imagegray = color_image[480:680, 170:350]
    # cv2.imwrite("/home/h/DeepClawBenchmark/Data/after.jpg", imagegray)
    # imagegray = color_image[820:1020, 300:450]
    # cv2.imshow('second_image', imagegray)

    compare = df.Segment()
    rect = compare.DiffGround(imagegray,imgray)
    if len(rect) > 0 :
        success_label = 1
        print('success_label:'+str(success_label))
        return success_label, imagegray
    else :
        success_label = 0
        print('success_label:'+str(success_label))

        return success_label, imagegray

if __name__=="__main__":
    a = success_label()
    print(a)