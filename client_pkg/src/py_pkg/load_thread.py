#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time

from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage

class Loader(QThread):
    loading = Signal(object)
    def __init__(self, parent = None):
        super(Loader, self).__init__()
        self.num = None
        self.text = "Loading"
        self.loading_flag = True

    def run(self):
        self.view_load()

    def view_load(self):
        self.num = 1
        while self.loading_flag:
            if self.num > 7:
                self.num = 1
            Loading = np.ones((70, self.num * 38, 3), dtype = np.float32) * 255
            newImage = cv2.putText(Loading, self.text[:self.num],
                                        (0, 50),
                                        cv2.FONT_HERSHEY_TRIPLEX,
                                        2,
                                        (0, 0, 0),
                                        2,
                                        cv2.LINE_AA)
            ros_Height, ros_Width, channel = newImage.shape
            newImage = cv2.cvtColor(newImage, cv2.COLOR_BGR2RGB)
            bytesPerLine = 3 * ros_Width
            newImage = newImage.astype('uint8')
            qt_image = QImage(newImage, ros_Width, ros_Height, bytesPerLine, QImage.Format_RGB888)

            self.loading.emit(qt_image)

            time.sleep(0.5)
            self.num += 1
