import numpy as np
import cv2


# class Calibration2D(object):
#     def __init__(self):
#         self._xy_set = ''
#         self._uv_set = ''
#         self._M_image2robot = ''
#
#     def matrix_update(self):
#         pts1 = np.float32(self._uv_set)
#         pts2 = np.float32(self._xy_set)
#         self._M_image2robot = cv2.getPerspectiveTransform(pts1,pts2)
#         print("Matrix updated!")
#
#     def cvt(self, u, v):
#         pick_point = [float(u), float(v)]
#         grasp_point = np.array([[pick_point]])
#         gp_base = cv2.perspectiveTransform(grasp_point, self._M_image2robot)
#         x = gp_base[0][0][0]
#         y = gp_base[0][0][1]
#         return [x, y]

class Calibration2D(object):
    #Todo: input the 4 pairs points
    def __init__(self):
        # UR10e
        # xy1 = [267.70,-155.49]
        # xy2 = [254.66,-542.36]
        # xy3 = [755.34,-172.14]
        # xy4 = [743.11,-556.34]
        #
        # uv1 = [412,195] #from image process
        # uv2 = [419,557]
        # uv3 = [883,191]
        # uv4 = [880,555]
        # UR5 for jigsaw
        xy1 = [-65.23,-436.58]
        xy2 = [-66.49,-681.45]
        xy3 = [216.14, -696.13]
        xy4 = [207.45,-464.67]

        uv1 = [656,284] #from image process
        uv2 = [658,502]
        uv3 = [916,516]
        uv4 = [908,304]

        #perspective transformation
        pts1 = np.float32([uv1,uv2,uv3,uv4])
        pts2 = np.float32([xy1,xy2,xy3,xy4])
        self.image2baseMatrix = cv2.getPerspectiveTransform(pts1,pts2)

    def cvt(self,u,v):
        pick_point = [u,v]
        grasp_point = np.array([[pick_point]], dtype=np.float32)
        gp_base = cv2.perspectiveTransform(grasp_point, self.image2baseMatrix)
        x = gp_base[0][0][0]
        y = gp_base[0][0][1]
        return x,y


if __name__ == '__main__':
    hand_eye = calibration()
    print(hand_eye.cvt(471,554))
