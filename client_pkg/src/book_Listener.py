#!/usr/bin/env python

import rospy

from std_msgs.msg import String
#app.exec_()
def callback(data):
    name = unicode(data.data, 'euc-kr').encode('utf-8')
    pub.publish(name)
    #print(name)



if __name__ == '__main__':
    rospy.init_node('book_listener', anonymous=True)
    pub = rospy.Publisher("/book_cpp_to_python", String, queue_size = 1)
    rospy.Subscriber('/book_cpp', String, callback)

    rospy.spin()
