import time
from typing import List, Optional, Tuple
import numpy as np
import cv2
import pyrealsense2 as rs
from threading import Event, Lock, Thread


def get_device_ids() -> List[str]:
    ctx = rs.context()
    devices = ctx.query_devices()
    device_ids = []
    for dev in devices:
        dev.hardware_reset()
        device_ids.append(dev.get_info(rs.camera_info.serial_number))
    time.sleep(2)
    return device_ids


class RealSenseCamera:
    def __repr__(self) -> str:
        return f"RealSenseCamera(device_id={self._device_id})"

    def __init__(self, device_id: Optional[str] = None, flip: bool = False):
        print("init", device_id)
        self._device_id = device_id
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(device_id)
        self.image = None

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 90)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 90)
        self._pipeline.start(config)
        self._flip = flip

        self.stop_thread = Event()
        self.lock = Lock()

        # thread1: wake up
        self.thread_wake_up = Thread(target=self.run_thread_read)
        self.thread_wake_up.daemon = True   # over with main part over
        self.thread_wake_up.start()
        time.sleep(2)

    def run_thread_read(self) :
        while not self.stop_thread.is_set():
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            # depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
            self.image = color_image
            depth = depth_image

    def shot_img(self, save_path):
        cv2.imwrite(save_path, self.image)



if __name__ == "__main__":
    device_ids = get_device_ids()
    print("device_ids: ",device_ids)
    rs = RealSenseCamera(flip=True, device_id="130322273839")
    while 1:
        cv2.imshow("demo", rs.image)
        cv2.waitKey(1)

