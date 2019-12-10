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
        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')  # opencv3.0
        # fourcc = cv2.VideoWriter_fourcc('M', 'P', '4', '2')
        if self.video_dir != '':
            videoWriter = cv2.VideoWriter(self.video_dir, fourcc, self.fps, self.image_size)

            while self.flag == 0:
                frame = self.camera.get_frame()
                videoWriter.write(frame.color_image[0])

            videoWriter.release()

    def stop(self):
        self.flag = 1
