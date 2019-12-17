#!/usr/bin/env python3
'''
#test
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
    launch = roslaunch.parent.ROSLaunchParent(uuid, [fixpath("~/cato/src/x2_cartographer/carto_mapper/launch/mapper.launch")])
    launch.start()
    #rospy.signal_shutdown("dd")
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

'''
import roslaunch

import rospy
import subprocess
import signal

from std_msgs.msg import Bool, String

class launch_manager():
    def __init__(self):
        self.flag = False
        rospy.Subscriber("/launch_select", String, self.launch_callback)
        rospy.spin()

    def load_map_mode(self):
        self.child = subprocess.Popen(["roslaunch","mapper_pkg","load_map_mode.launch"])
        #child.wait() #You can use this line to block the parent process untill the child process finished.
        #print("parent process")
        #print(self.child.poll())

        #rospy.loginfo('The PID of child: %d', self.child.pid)
        #print ("The PID of child:", self.child.pid)

    def create_map_mode(self):
        self.child = subprocess.Popen(["roslaunch","mapper_pkg","create_map_mode.launch"])

    def launch_callback(self, data):
        if self.flag == True :
            if data.data == "load_map_mode_close":

                self.child.send_signal(signal.SIGINT)
                self.flag = False
            elif data.data == "create_map_mode_save":
                pass

        else:

            if data.data == "load_map_mode" :
                self.flag = True
                self.load_map_mode()

            elif data.data == "create_map_mode" :
                self.flag = True
                self.create_map_mode()


if __name__ == "__main__":
    rospy.init_node("launch_manager", anonymous=True)
    print("Start Process")
    launch_manager()
    print("End Process")
