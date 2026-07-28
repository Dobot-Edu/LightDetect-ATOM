import time
from typing import List, Optional, Tuple
import numpy as np
import cv2
import pyrealsense2 as rs
from threading import Event, Lock, Thread


def get_available_cameras(max_cameras=10):
    available = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available.append(i)
        cap.release()
    return available


class UsbCamera:
    def __repr__(self) -> str:
        return f"UsbCamera(device_id={self._device_id})"

    def __init__(self, device_id:Optional[int] = None, flip: bool = False):
        print("init", device_id)
        self._device_id = device_id
        self.cap = cv2.VideoCapture(device_id)
        self.image = None

        self.stop_thread = Event()
        self.lock = Lock()

        # thread1: wake up
        self.thread_wake_up = Thread(target=self.run_thread_read)
        self.thread_wake_up.daemon = True   # over with main part over
        self.thread_wake_up.start()
        time.sleep(2)

    def run_thread_read(self) :
        while not self.stop_thread.is_set():
            ret, color_image = self.cap.read()
            self.image = color_image

    def shot_img(self, save_path):
        cv2.imwrite(save_path, self.image)



if __name__ == "__main__":
    print(get_available_cameras())
    rs = UsbCamera(flip=True, device_id=2)
    while 1:
        cv2.imshow("demo", rs.image)
        cv2.waitKey(1)

