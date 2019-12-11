#!/usr/bin/env python
import numpy as np
import cv2
from matplotlib import pyplot as plt
MIN_MATCH_COUNT = 4

def siftMatch(sourceImage,targetImage,showPicture = False):
    #cv2.xfeatures2d.SIFT_create(int nfeature=0,int nOctaveLayers=3,double contrastThreshold=0.04,double edgeThreshold=10,double sigma=1.6)
    #nfeature: The number of best features to retain
    #nOctaveLayers: The number of layers in each octave
    #contrastThreshold: used to filter out weak features in semi-uniform (low-contrast) regions
    #edgeThreshold: used to filter out edge-like features,the larger the edgeThreshold, the less features are filtered out (more features are retained)
    #sigma: The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number
    sift = cv2.xfeatures2d.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(sourceImage,None)
    kp2, des2 = sift.detectAndCompute(targetImage,None)
    if(len(kp2) == 0 or len(kp1)==0):
        return 0,None
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.8*n.distance:
            good.append(m)
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        if(M is None):
                return len(good),None
        matchesMask = mask.ravel().tolist()
        angle = np.arctan2(M[1,0],M[0,0])
        # print angle * 180.0/3.1415926,"degrees"

        # h,w,_ = sourceImage.shape
        imageSize = sourceImage.shape
        h = imageSize[0]
        w = imageSize[1]
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        targetImage = cv2.polylines(targetImage,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        if(showPicture == True):
            draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                               singlePointColor = None,
                               matchesMask = matchesMask, # draw only inliers
                               flags = 2)
            img3 = cv2.drawMatches(sourceImage,kp1,targetImage,kp2,good,None,**draw_params)
            plt.imshow(img3, 'gray'),plt.show()
        return len(good),angle
    else:
        print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
        matchesMask = None
        return len(good),None

if __name__ == '__main__':
    srcImage =cv2.imread('/home/bionicdl-razer/git-projects/finetune_alexnet_with_tensorflow/jigsaw_images/0.jpg',0)
    targetImage =cv2.imread('/home/bionicdl-razer/git-projects/finetune_alexnet_with_tensorflow/jigsaw_images/2.jpg',0)
    matchNum,angle = siftMatch(srcImage,targetImage)
    print("angle: ",angle)
