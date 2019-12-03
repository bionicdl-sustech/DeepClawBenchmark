# from driver.sensors.camera.AKinectController import AKinectController
# camera = AKinectController()
from driver.sensors.camera.RealsenseController import RealsenseController
camera = RealsenseController()

import cv2
import numpy as np
from modules.localization.contour_filter import contour_filter
from modules.recognition.color_recognition import color_recognition
operator = contour_filter(area_threshold=[900, 1050])
blue_recognitor = color_recognition([100, 43, 46], [124, 255, 255])
green_recognitor = color_recognition([35, 43, 46], [77, 255, 255])

while 1:
	current_frame = camera.get_frame()
	color_image = current_frame.color_image[0]
	background = np.zeros((color_image.shape[0], color_image.shape[1]))
	# sub_image = color_image[300:550, 450:850, :]
	sub_image = color_image[100:550, 450:850, :]
	bbox, mask, centers = operator.display(sub_image)
	cv2.imshow("image", sub_image)
	cv2.waitKey(1)
	cv2.imshow("mask", mask)
	cv2.waitKey(1)
	# if len(centers)!=0:
	# 	labels, _ = green_recognitor.display(centers, sub_image)
	# 	print(labels)
		#piece = sub_image[centers[0][1]-10:centers[0][1]+10, centers[0][0]-10:centers[0][0]+10, :]
		# print(np.mean(piece[:, :, 0]), np.mean(piece[:, :, 1]), np.mean(piece[:, :, 2]))

	# display = cv2.bitwise_and(color_image, color_image, mask=background)
	# (100, 60, 78)
	# (61, 85, 39) green
	# 

		# cv2.imshow("mask", piece)
		# cv2.waitKey(1)