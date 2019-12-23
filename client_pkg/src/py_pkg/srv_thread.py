#!/usr/bin/env python3

import sys
import rospy
import os
import roslaunch

from datetime import datetime
#from client_pkg.srv import *
from cartographer_ros_msgs.srv import *
from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage

def fixpath(path):
    return os.path.abspath(os.path.expanduser(path))

class Server(QThread):
    send_server_data = Signal()
    def __init__(self, parent=None):
        super(Server, self).__init__()
        self.parent = parent

    def run(self):
        self.map_saver()

    def map_saver(self):
        rospy.wait_for_service('finish_trajectory')
        try:
            req = rospy.ServiceProxy('finish_trajectory', FinishTrajectory)
            res = req(0)
            #print(res.status)
            self.parent.ui.statusbar.showMessage(res.status.message)
            if(res.status.code == 0 or res.status.code == 8):
                rospy.wait_for_service('write_state')
                try:
                    req = rospy.ServiceProxy('write_state', WriteState)
                    #msg = '../catkin_ws/src/bitproject/mapper_pkg/map_data/map.bag.pbstream'
                    res = req('../catkin_ws/src/bitproject/mapper_pkg/map_data/map.bag.pbstream')
                    #print(res.status)
                    self.parent.ui.statusbar.showMessage(res.status.message + res.status.code)
                    if(res.status.code == 0):
                        self.parent.ui.statusbar.showMessage("Map Save Success!")
                        self.send_server_data.emit()
                except rospy.ServiceException as e:
                    print ("Service call failed: %s"%(e))

                finally:
                    self.parent.ui.statusbar.showMessage("Map Save!")
                    self.send_server_data.emit()
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%(e))



if __name__ == "__main__":

    print ("Requesting ")
    print ("%s"%(add_two_ints_client()))
