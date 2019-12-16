#!/usr/bin/env python3

import os
import roslaunch
import sys
import rospy
from py_test.srv import *

global flag
flag = 0
def fixpath(path):
    return os.path.abspath(os.path.expanduser(path))
def open_launch():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    print("uuid", uuid)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [fixpath("~/carto_x2/src/x2_cartographer/carto_mapper/launch/mapper.launch")])
    rospy.signal_shutdown()
    launch.start()
    rospy.loginfo("started")

def handle_add_two_ints(req):
    global flag
    resmsg = "ohohohohohohohohohohOh!"
    if(flag == 0):
        flag = 1
        open_launch()

    #rospy.sleep(30000)
    # 3 seconds later
    #launch.shutdown()
    return Main_SrvResponse(resmsg)

def add_two_ints_server():
    s = rospy.Service('map_trigger', Main_Srv, handle_add_two_ints)
    print ("Ready to add two ints.")

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('add_two_ints_server', anonymous=True)
    add_two_ints_server()
