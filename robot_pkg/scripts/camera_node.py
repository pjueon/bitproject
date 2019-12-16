#!/usr/bin/env python

import sys
import rospy
import cv2
import threading
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen
class camera_Worker(threading.Thread):
    def __init__(self):
        self.flag = False
        self.wait_flag = threading.Event()
        threading.Thread.__init__(self)
        self.image_pub = rospy.Publisher("/camera_topic", CompressedImage, queue_size = 1)#encoding defult jpg
        self.bridge = CvBridge()

    def run(self):
        # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
        while not rospy.is_shutdown():
            if self.flag:
                print("Starting Camera")
                self.camera()

    def camera(self):
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if self.cap.isOpened():

            while self.flag:
                self.ret_val, self.img = self.cap.read()
                #image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))#none encoding
                self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(self.img))#encoding defult jpg
                self.wait_flag.wait()# Python Threading.Event flag가 True가 될 때까지 기다림 초기 Flag는 True이기 때문에 넘어감
            print("Close Camera")

        else:
            print("Unable to open camera")

        self.cap.release()
        print(self.cap.isOpened())

def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=10,
    flip_method=2
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def callback(data):
    try:
        if toggle.is_alive():
            toggle.flag = data.data
            print("callback data : ", data.data)
            if data.data and not toggle.wait_flag.is_set():
                toggle.wait_flag.set()# Threading.Event의 Flag를 True로 set함
            elif not data.data and toggle.wait_flag.is_set():
                toggle.wait_flag.clear()# Threading.Event의 Flag를 False로 clear함
            else:
                pass
    except:
        pass

if __name__ == "__main__":
    rospy.init_node("camera_node")
    toggle = camera_Worker()
    toggle.daemon = True
    toggle.start()
    rospy.Subscriber("/camera_toggle", Bool, callback)
    print("Starting Thread")
    rospy.spin()
    print("End Process")
    #image_pub = rospy.Publisher("image_topic", Image)#none encoding
