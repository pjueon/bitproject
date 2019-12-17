import rospy
import numpy as np
import cv2
import time

from sensor_msgs.msg import CompressedImage, Image
from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage
from cv_bridge import CvBridge, CvBridgeError

class Worker(QThread):
    send_camera_view = Signal(object)
    def __init__(self, parent=None, tfnet=None):
        super(Worker, self).__init__()
        #self.main = parent
        self.tfnet = tfnet
        self.bridge = CvBridge()

    def __del__(self):
        del self.bridge
        del self.tfnet
        del self
        print("============================= End Camera Thread ================================\n\n")

    def run(self):
        rospy.Subscriber("/camera_topic", CompressedImage, self.callback)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(data)# encoding
            self.results = self.tfnet.return_predict(self.cv_image)
            self.cv_image = self.boxing(self.cv_image, self.results)
            self.cv_image = cv2.resize(self.cv_image, dsize=(970, 630))
            self.height, self.width, self.channel = self.cv_image.shape
            self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

            self.bytesPerLine = 3 * self.width
            self.qt_image = QImage(self.cv_image, self.width, self.height, self.bytesPerLine, QImage.Format_RGB888)

            self.pixmap = QPixmap.fromImage(self.qt_image)
            self.send_camera_view.emit(self.pixmap)

        except CvBridgeError as e:
            print("CvBridge err", e)
            pass

    def boxing(self, original_img, predictions):
        self.newImage = np.copy(original_img)

        for result in predictions:
            self.top_x = result['topleft']['x']
            self.top_y = result['topleft']['y']

            self.btm_x = result['bottomright']['x']
            self.btm_y = result['bottomright']['y']

            self.confidence = result['confidence']
            self.label = result['label'] + " " + str(round(self.confidence, 3))

            if self.confidence > 0.6:
                self.newImage = cv2.rectangle(self.newImage, (self.top_x, self.top_y), (self.btm_x, self.btm_y), (255,0,0), 3)
                self.newImage = cv2.putText(self.newImage, self.label, (self.top_x, self.top_y-5), cv2.FONT_HERSHEY_COMPLEX_SMALL , 0.8, (0, 230, 0), 1, cv2.LINE_AA)

        return self.newImage
