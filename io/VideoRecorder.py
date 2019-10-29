import cv2
import os

class VideoRecorder(object):
    def __init__(self, camera, fps=30, image_size=(1280, 720)):
        self.flag = 0
        self.fps = fps
        self.image_size = image_size
        self.camera = camera
        self.video_dir = ''

    def start(self):
        # fourcc = cv2.cv.CV_FOURCC('M','J','P','G')#opencv2.4
        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')  # opencv3.0
        if self.video_dir!='':
            videoWriter = cv2.VideoWriter(self.video_dir, fourcc, self.fps, self.image_size)

            while self.flag==0:
                frame, _ = self.camera.getImage()
                videoWriter.write(frame)

            videoWriter.release()

    def stop(self):
        self.flag = 1
