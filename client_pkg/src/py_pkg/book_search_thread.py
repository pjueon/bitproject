#!/usr/bin/env python3
import rospy
import sys
import std_msgs
from std_msgs.msg import String
from PySide2.QtWidgets import QLabel
from PySide2.QtCore import QThread, Signal, Slot
from nav_msgs.msg import MapMetaData, OccupancyGrid

class Book_search(QThread):
    def __init__(self, parent = None):
        super(Book_search, self).__init__()
        self.parent = parent
        self.sub = None

    def run(self):
        self.sub = rospy.Subscriber("/book_cpp_to_python", std_msgs.msg.String, self.callback)

    def __del__(self):
        if not self.sub == None:
            self.sub.unregister()
        print("============================= End Book_search Thread ===========================\n\n")

    def callback(self, data):
        self.parent.ui.listwidget.addItem(data.data)
        self.parent.ui.listwidget.scrollToBottom()

def main(args):
  rospy.Subscriber("/book_cpp_to_python", std_msgs.msg.String, self.callback)
  rospy.init_node('simple_class')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
