            #!/usr/bin/env python3
import cv2
import numpy as np
import depthai as dai
import threading
import pyvirtualcam

class Webcam:
    def __init__(self):
        pass

    def create_pipeline(self):
        print("Creating pipeline: RGB CAM -> XLINK OUT")
        pipeline = dai.Pipeline()

        cam          = pipeline.createColorCamera()
        xout_video   = pipeline.createXLinkOut()

        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        
        xout_video.setStreamName('rgb_video')
        cam.video.link(xout_video.input)

        streams = ['rgb_video']

        return pipeline, streams

    def pipe_to_virtual_webcam(self, device):
        self.rgb_video = device.getOutputQueue(name="rgb_video", maxSize=4, blocking=False)
        name  = self.rgb_video.getName()
        image = self.rgb_video.get()
        data, w, h = image.getData(), image.getWidth(), image.getHeight()
        with pyvirtualcam.Camera(width=w, height=h, fps=30) as cam:
            print(f'Using virtual camera: {cam.device}')
            while True:
                self.rgb_video = device.getOutputQueue(name="rgb_video", maxSize=4, blocking=False)
                name  = self.rgb_video.getName()
                image = self.rgb_video.get()
                frame = self.convert_to_cv2_frame(name, image)
                cam.send(frame)
                cam.sleep_until_next_frame()

    def run(self):
        pipeline, _ = self.create_pipeline()
     
        with dai.Device(pipeline) as device:
            # Start pipeline
            device.startPipeline()
            self.pipe_to_virtual_webcam(device)

    def convert_to_cv2_frame(self, name, image):
        data, w, h = image.getData(), image.getWidth(), image.getHeight()
        # TODO check image frame type instead of name
        if name == 'rgb_video': # YUV NV12
            yuv = np.array(data).reshape((h * 3 // 2, w)).astype(np.uint8)
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_NV12)
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            frame = cv2.flip(frame, flipCode=1)
        return frame


if __name__ == '__main__':
    webcam = Webcam()
    webcam.run()


