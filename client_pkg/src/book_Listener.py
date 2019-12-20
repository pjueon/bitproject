#!/usr/bin/env python

import rospy

from std_msgs.msg import String
#app.exec_()
def callback(data):
    name = unicode(data.data, 'euc-kr').encode('utf-8')
    pub.publish(name)

def listener():

    rospy.init_node('listr', anonymous=True)
    rospy.Subscriber('/book_cpp', String, callback)

if __name__ == '__main__':
    pub = rospy.Publisher("/book_cpp_to_python", String, queue_size = 1)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
