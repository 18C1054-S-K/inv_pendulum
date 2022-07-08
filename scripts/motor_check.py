#!/usr/bin/env python3

import time
import rospy
import gpiozero

if __name__ == "__main__":
	rospy.init_node('motor_check', anonymous=True)
	motorL = gpiozero.Motor(forward=22, backward=27)
	motorR = gpiozero.Motor(forward=24, backward=23)

	time.sleep(2.0)
	v = rospy.get_param('/motor_check/volt')
	o = v / 9.0
	motorL.forward(o)
	motorR.forward(o)
	print('output : ', o*100.0, '%')
	while not rospy.is_shutdown():
		time.sleep(1.0)

