#!/usr/bin/env python3

import time
import rospy

if __name__ == "__main__":
	rospy.init_node('test', anonymous=True)
	pub = rospy.Publisher('tire_angv', Float32, queue_size=10)

	angv = 0.0
	while not rospy.is_shutdown():
		angv += 0.2
		if angv > 1.0:
			break
		pub.publish(angv * 3.14159)
		time.sleep(1.0)

	time.sleep(3.0)
	
	while not rospy.is_shutdown():
		angv -= 0.2
		if angv < 0.0:
			pub.publish(0.0)
			break
		pub.publish(angv * 3.14159)
		time.sleep(1.0)
	
		
