#!/usr/bin/env python3
import rospy
import math
import tf
import cv2
import numpy as np

from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose

from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage

class Mapper(QThread):
    send_map_view = Signal(object)
    def __init__(self, parent=None):
        super(Mapper, self).__init__()
        self.ros_Width = 0
        self.ros_Height = 0
        self.origin_x = 0
        self.origin_y = 0
        self.resoution = 0.
        self.x = 0.
        self.y = 0.
        self.yew = 0.
        self.resizingFactor = 1.
        self.markerSize = 14
        self.listener = tf.TransformListener()
        self.PosMarker = np.ones((14, 14, 3), dtype = np.float32)*255
        self.PosMarker = cv2.arrowedLine(self.PosMarker, (0,
                                                          int(self.markerSize/2)),
                                                          (self.markerSize, int(self.markerSize/2)),
                                                          (0, 0, 255),
                                                          2,
                                                          2,
                                                          0,
                                                          0.5)

    def run(self):
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)

    def __del__(self):
        print("============================= End Map Reader Thread ============================")

    def mapCallback(self, data):

        self.ros_Width = data.info.width
        self.ros_Height = data.info.height
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        self.resolution = data.info.resolution

        self.updatePosition()
        map = self.drawMap(data.data)
        map = self.mapResize(map)
        map = self.plotPosMarker(map)

        map = cv2.flip(map, 0)
        ros_Height, ros_Width, channel = map.shape
        map = cv2.cvtColor(map, cv2.COLOR_BGR2RGB)
        bytesPerLine = 3 * ros_Width
        qt_image = QImage(map, ros_Width, ros_Height, bytesPerLine, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.send_map_view.emit(pixmap)

    def updatePosition(self):
        while not rospy.is_shutdown():
            try:
                self.transform, self.rot = self.listener.lookupTransform("/map",
                                                                         "/base_footprint",
                                                                         rospy.Time(0)
                                                                         )
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.yaw = tf.transformations.euler_from_quaternion(self.rot)[2]
        self.x = self.transform[0]
        self.y = self.transform[1]

    def drawMap(self, data):
        block_threshold = 50
        slope = (128. - 255.) / block_threshold

        #    pass
        def mapfnc(value):
            if(value == -1): # unknown
                return 100
            elif(value > block_threshold): # blocked
                return 0
            else: # 0 ~ blocK_threshold
                return int(slope * value + 255);
            # 128 <= color <= 255
        drawmap = map(mapfnc, data)
        drawmap = np.reshape(list(drawmap), (self.ros_Height, self.ros_Width))
        drawmap = drawmap.astype('uint8')
        return drawmap

    def mapResize(self, map):
        LongerSide = 950.
        self.resizingFactor = LongerSide / map.shape[0] if map.shape[0] > map.shape[1] else LongerSide / map.shape[1]
        self.resizingFactor = self.resizingFactor
        width = int(self.resizingFactor * map.shape[1])
        height = int(self.resizingFactor * map.shape[0])
        dim = (width, height)
        map = cv2.resize(map.astype('uint8'), dim, interpolation=cv2.INTER_AREA)
        map = np.stack((map,)*3, axis=-1)# 1channel -> 3channel
        return map

    def plotPosMarker(self, map):
        RotatedMarker = self.arrowedLine_Rotate()
        RotatedMarker = RotatedMarker.astype('uint8')
        markerWidth, markerHeight, markerChannel = RotatedMarker.shape

#================================Select ROI==========================================
        cell_x = round((self.x - self.origin_x) / self.resolution * self.resizingFactor)
        cell_y = round((self.y - self.origin_y) / self.resolution * self.resizingFactor)

        roi_x_start = int(cell_x-markerWidth/2)
        roi_x_end = int(cell_x-markerWidth/2) + markerWidth

        roi_y_start = int(cell_y-markerHeight/2)
        roi_y_end = int(cell_y-markerHeight/2) + markerHeight
        roi = map[roi_y_start : roi_y_end, roi_x_start : roi_x_end]

#===============================ArrowedLine Masking===================================

        gray_Marker = cv2.cvtColor(RotatedMarker, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(gray_Marker, 200, 255, cv2.THRESH_BINARY_INV)
        mask = mask.astype('uint8')
        mask_inv = cv2.bitwise_not(mask)
        mask_inv = mask_inv.astype('uint8')
        #mask = np.stack((mask,)*3, axis=-1)
        #print("RotatedMarker", type(RotatedMarker), RotatedMarker.shape, RotatedMarker.dtype)
        #print("mask         ", type(mask), mask.shape, mask.dtype)
        #print("mask_inv     ", type(mask_inv), mask_inv.shape, mask_inv.dtype)
        #print("map          ", type(map), map.shape, map.dtype)
        #print("roi          ", type(roi), roi.shape, roi.dtype)
        arrow_fg = cv2.bitwise_and(RotatedMarker, RotatedMarker, mask = mask)
        arrow_fg = arrow_fg.astype('uint8')

        map_fg = cv2.bitwise_and(roi, roi, mask = mask_inv)
        #print("arrow_fg     ", type(arrow_fg), arrow_fg.shape, arrow_fg.dtype)
        #print("map_fg       ", type(map_fg), map_fg.shape, map_fg.dtype)
        dst = cv2.add(arrow_fg, map_fg)

        map[roi_y_start : roi_y_end, roi_x_start : roi_x_end] = dst
        return map

    def arrowedLine_Rotate(self):
        angle = -self.yaw * 180. / np.pi
        height, width, channel = self.PosMarker.shape
        MarkerCenter = (int(width/2), int(height/2))

        RotatedMarker = cv2.getRotationMatrix2D(MarkerCenter, angle, 1.)

        abs_cos = abs(RotatedMarker[0,0])
        abs_sin = abs(RotatedMarker[0,1])

        bound_w = int(height * abs_sin + width * abs_cos)
        bound_h = int(height * abs_cos + width * abs_sin)

        # subtract old image center (bringing image back to origo) and adding the new image center coordinates
        RotatedMarker[0, 2] += bound_w/2 - MarkerCenter[0]
        RotatedMarker[1, 2] += bound_h/2 - MarkerCenter[1]

        # rotate image with the new bounds and translated rotation matrix
        RotatedMarker = cv2.warpAffine(self.PosMarker,
                                       RotatedMarker,
                                       (bound_w, bound_h),
                                       borderValue = (255, 255, 255)
                                       )
        return RotatedMarker

if __name__ == '__main__':
    rospy.init_node("map_reader")
    map_reader = Mapper()
    map_reader.updatePosition()
    rospy.spin()
