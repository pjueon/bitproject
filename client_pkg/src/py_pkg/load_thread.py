#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time
import os

from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage

def fixpath(path):
    return os.path.abspath(os.path.expanduser(path))

class Loader(QThread):
    loading = Signal(object)
    def __init__(self, parent = None):
        super(Loader, self).__init__()
        self.num = 360
        self.text = "Loading..."
        self.loading_flag = True

    def run(self):
        self.view_load()

    def __del__(self):
        print("============================= End loading Thread ===============================\n\n")

    def view_load(self):
        newImage = cv2.imread(fixpath("~/catkin_ws/src/bitproject/client_pkg/src/py_pkg/image/loading.png"), cv2.IMREAD_COLOR)

        newImage = cv2.cvtColor(newImage, cv2.COLOR_BGR2RGB)
        ros_Height, ros_Width, channel = newImage.shape
        bytesPerLine = 3 * ros_Width
        Center = (int(ros_Width/2), int(ros_Height/2))
        newImage = newImage.astype('uint8')
        while self.loading_flag:
            '''
            if self.num > 10:
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
            '''
            if self.num < 0:
                self.num = 360
            #newImage = cv2.cvtColor(newImage, cv2.COLOR_BGR2RGB)
            angle = self.num

            Rotated = cv2.getRotationMatrix2D(Center, angle, .3)

            #abs_cos = abs(Rotated[0,0])
            #abs_sin = abs(Rotated[0,1])

            #bound_w = int(ros_Height * abs_sin + ros_Width * abs_cos)
            #bound_h = int(ros_Height * abs_cos + ros_Width * abs_sin)

            # subtract old image center (bringing image back to origo) and adding the new image center coordinates
            #Rotated[0, 2] += bound_w/2 - Center[0]
            #Rotated[1, 2] += bound_h/2 - Center[1]

            # rotate image with the new bounds and translated rotation matrix
            RotatedMaker = cv2.warpAffine(newImage,
                                           Rotated,
                                           (ros_Width, ros_Height),
                                           borderValue = (255, 255, 255)
                                           )
            qt_image = QImage(RotatedMaker, ros_Width, ros_Height, bytesPerLine, QImage.Format_RGB888)
            #print(angle)
            self.loading.emit(qt_image)
            time.sleep(0.05)
            self.num -= 30
