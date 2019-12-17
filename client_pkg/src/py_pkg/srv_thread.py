#!/usr/bin/env python3

import sys
import rospy
import os
import roslaunch

from client_pkg.srv import *
from cartographer_ros_msgs.srv import *
from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage

class Server(QThread):
    def __init__(self, parent=None):
        super(Server, self).__init__()
        self.parent = parent

    def add_trigger_init_client(self):

        rospy.wait_for_service('finish_trajectory')
        print("waiting srv")
        try:
            res = rospy.ServiceProxy('finish_trajectory', FinishTrajectory)
            res = res(0)
            print(res.status)
            self.parent.ui.statusbar.showMessage(res.status.message)
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%(e))
    '''
    def add_trigger_init_client(self):
        rospy.wait_for_service('map_trigger')
        try:
            add_two_ints = rospy.ServiceProxy('map_trigger', Main_Srv)
            res = add_two_ints()
            print(res.resmsg)
            self.parent.ui.statusbar.showMessage(res.resmsg)

        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
    '''

if __name__ == "__main__":

    print ("Requesting ")
    print ("%s"%(add_two_ints_client()))
