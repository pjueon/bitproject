#!/usr/bin/env python3

print("[lsm303] importing modules...")
import rospy
import time
import board
import busio
import tf
import math
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Imu
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as generic_linux_I2C


i2c = busio.I2C(board.SCL, board.SDA)
# force to use bus 0
i2c._i2c = generic_linux_I2C(0, mode=0, baudrate=400000)

print("[lsm303] initializing sensors...")
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)


def fitAngleInRange(angle):
	n = math.floor((math.pi - angle)/(2*math.pi))
	return angle + 2*math.pi*n

def start():
	# ROS IMU 전달부 세팅 #
	#static_transform = rospy.get_param('~static_transform', [0, 0, 0, 0, 0, 0])
	fixed_frame = rospy.get_param('~fixed_frame', "base_footprint")
	frame_name = rospy.get_param('~frame_name', "imu")

	rospy.init_node('lsm303',anonymous=True)
	pub = rospy.Publisher('imu',Imu,queue_size=1)
	odomBroadcaster_imu = TransformBroadcaster()
	frequency = 5
	r = rospy.Rate(frequency)

	data = Imu()
	seq = 0
	
	print("[lsm303] loop start!")
	while not rospy.is_shutdown():
		data.header.frame_id = "imu"
		data.header.stamp = rospy.get_rostime()
		data.header.stamp = rospy.Time.now()
		data.header.seq = seq

		data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z = accel.acceleration
		#print("acceleration: {}".format(accel.acceleration))

		mag_x, mag_y, mag_z = mag.magnetic
		theta = -math.atan2(mag_y, mag_x)		
		#print("theta: {}".format(theta))

		q = tf.transformations.quaternion_from_euler(0, 0, theta)
		data.orientation.x = q[0]
		data.orientation.y = q[1]
		data.orientation.z = q[2]
		data.orientation.w = q[3]
		
		odomBroadcaster_imu.sendTransform(
			(0, 0, 0),
			(0, 0, 0, 1),
			rospy.Time.now(), frame_name, fixed_frame)

		#print("q(x,y,z,w): ({}, {}, {}, {})".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))

		data.angular_velocity.x = 0
		data.angular_velocity.y = 0
		data.angular_velocity.z = 0
					
		seq += 1
		pub.publish(data)
		r.sleep()
	
	print("[lsm303] program end")



if __name__=='__main__':
	start()

