#!/usr/bin/env python3

print("===================\n[motor_control]importing modules...")

import rospy
from std_msgs.msg import String

import time
from jetbot import Robot

robot = Robot()
SPEED = 0.25
ROTATION_SPEED = 0.175

def callback(msg):
    cmd = msg.data.upper()

    if cmd == "F":
        robot.forward(SPEED)
    elif cmd == "B":
        robot.backward(SPEED)
    elif cmd == "L":
        robot.left(ROTATION_SPEED)
    elif cmd == "R":
        robot.right(ROTATION_SPEED)
    elif cmd == "FL":
        robot.set_motors(0.75*SPEED, SPEED)
    elif cmd == "FR":
        robot.set_motors(SPEED, 0.75*SPEED)
    elif cmd == "BL":
        robot.set_motors(-0.75*SPEED, -SPEED)
    elif cmd == "BR":
        robot.set_motors(-SPEED, -0.75*SPEED)
    else:
        robot.stop()

def main():
    rospy.init_node("motor_control_node")
    rospy.Subscriber("/motor", String, callback, queue_size=1)
    print("[motor_control]ready to move\n===================\n")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except:
        pass
