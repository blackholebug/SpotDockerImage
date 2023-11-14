import cv2
import numpy as np
from tqdm import tqdm
import threading

import bosdyn.util
from bosdyn.api import image_pb2, image_service_pb2_grpc
from bosdyn.client.image import ImageClient, build_image_request, save_images_as_files



class VideoStreamSaver:
    
    def __init__(self, image_client, quality_percent=90):
        self.image_client = image_client
        self.images_to_be_saved = [
            "hand_color_image",
            "back_fisheye_image"
            # "left_fisheye_image",
            # "right_fisheye_image",
            # "frontleft_fisheye_image",
            # "frontright_fisheye_image",
        ]
        
        image_format = image_pb2.Image.FORMAT_JPEG
        pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
        
        self.image_requests = [build_image_request(x, quality_percent=quality_percent, image_format=image_format, pixel_format=pixel_format) for x in self.images_to_be_saved]
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')  # or 'MP4V', 'MJPG', etc.
        self.video_writers = [
            cv2.VideoWriter(f"./data/images/hand_color.avi", fourcc, 2.0, (640,480)),
            cv2.VideoWriter(f"./data/images/back_fisheye.avi", fourcc, 2.0, (640,480))
        ]

    def video_thread_with_image_client(self):
        image_responses = self.image_client.get_image(self.image_requests)
        
        i = 0
        for image in image_responses:
            img = np.frombuffer(image.shot.image.data, dtype=np.uint8)
            img = cv2.imdecode(img, -1)
            self.video_writers[i].write(img)
            i += 1


if __name__ == "__main__":
    # Create and authenticate a bosdyn robot object.
    sdk = bosdyn.client.create_standard_sdk('ImageServiceSDK')
    robot = sdk.create_robot("192.168.1.109")
    bosdyn.client.util.authenticate(robot)

    robot.time_sync.wait_for_sync()
    image_client = robot.ensure_client(ImageClient.default_service_name)
        
    vss = VideoStreamSaver(image_client, 90)
    for i in tqdm(range(10)):
        vss.video_thread_with_image_client()
        
        cv2.waitKey(20)
        
        
    for out in vss.video_writers:
        out.release()