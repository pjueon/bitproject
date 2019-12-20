import rospy
from std_msgs.msg import String
from PySide2.QtWidgets import QWidget, QMainWindow
from PySide2.QtCore import QThread, Signal, Slot, Qt, QEvent
class create_map(QMainWindow, QWidget):
    def __init__(self):
        super(create_map, self).__init__()
        self.create_pub = rospy.Publisher("/motor", String, queue_size = 1)
    def keyEvent(self, event):
        # Front 87 Right 68 Left 65 Back 83
        if event.type() == QEvent.KeyPress:

            print("press")
            if(event.key() == 68 and event.key() == 87):
                self.create_pub.publish("FR")
            elif(event.key() == 65 and event.key() == 87):
                self.create_pub.publish("FL")
            elif(event.key() == 87):
                self.create_pub.publish("F")
            elif(event.key() == 68):
                self.create_pub.publish("R")
            elif(event.key() == 65):
                self.create_pub.publish("L")
            elif(event.key() == 83):
                self.create_pub.publish("B")
        if event.type() == QEvent.KeyRelease:
            print("release")
            self.create_pub.publish("S")

    def stop_motor(self):
        print("release")
        self.create_pub.publish("S")
