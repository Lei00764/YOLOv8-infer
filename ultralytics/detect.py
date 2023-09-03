# @time   : 2023/7/22 10:23
# @author : Xiang Lei
# opencv自己有一个缓存，每次会顺序从自己的缓存中读取，而不是直接读取最新帧
# 单独用一个线程实时捕获视频帧，主线程在需要时从子线程拷贝最近的帧使用即可

import time
import cv2
import threading
import numpy as np
from copy import deepcopy

from ultralytics import YOLO

thread_lock = threading.Lock()  # 线程锁
thread_exit = False  # 线程退出标志


class MyThread(threading.Thread):
    def __init__(self, camera_id, img_height, img_width):
        super().__init__()
        self.camera_id = camera_id
        self.img_height = img_height
        self.img_width = img_width
        self.frame = np.zeros((img_height, img_width, 3), dtype=np.uint8)

    def get_frame(self):
        return deepcopy(self.frame)

    def run(self):
        global thread_exit
        cap = cv2.VideoCapture(self.camera_id)
        while not thread_exit:
            ret, frame = cap.read()
            if ret:
                frame = cv2.resize(frame, (self.img_width, self.img_height))
                thread_lock.acquire()
                self.frame = frame
                thread_lock.release()
            else:
                thread_exit = True
        cap.release()


def main():
    global thread_exit
    model = YOLO('weights/best.pt')
    video_file = "try.mp4"

    # model = YOLO("yolov8n.pt")
    # video_file = 0
    camera_id = video_file
    img_height = 640
    img_width = 640

    thread = MyThread(camera_id, img_height, img_width)  # 创建线程
    thread.start()  # 启动线程

    while not thread_exit:
        thread_lock.acquire()
        frame = thread.get_frame()
        thread_lock.release()

        results = model(frame)
        annotated_frame = results[0].plot()

        cv2.imshow("frame", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            thread_exit = True


if __name__ == "__main__":
    main()
