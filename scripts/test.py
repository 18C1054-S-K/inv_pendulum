#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":
	rospy.init_node('test', anonymous=True)
	pub = rospy.Publisher('tire_angv', Float32, queue_size=10)

	if not rospy.is_shutdown():
		print('f slow')
		pub.publish(3.14159 / 2.0)
	time.sleep(1.0)

	if not rospy.is_shutdown():
		print('f fast')
		pub.publish(3.14159)
	time.sleep(1.0)

	if not rospy.is_shutdown():
		print('f slow')
		pub.publish(3.14159 / 2.0)
	time.sleep(1.0)

	if not rospy.is_shutdown():
		print('stop')
		pub.publish(0.0)
	time.sleep(1.0)
	
	if not rospy.is_shutdown():
		print('b slow')
		pub.publish(-3.14159 / 2.0)
	time.sleep(1.0)
	
	if not rospy.is_shutdown():
		print('stop')
		pub.publish(0.0)

	print('finish')
		
