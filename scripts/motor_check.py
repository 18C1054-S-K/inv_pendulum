#!/usr/bin/env python3

import time
import rospy
import gpiozero

if __name__ == "__main__":
	rospy.init_node('motor_check', anonymous=True)
	motorL = gpiozero.Motor(forward=27, backward=22)
	motorR = gpiozero.Motor(forward=23, backward=24)

	v = rospy.get_param('/motor_check/volt')
	o = v / 9.0
	motorL.forward(o)
	motorR.forward(o)
	print('output : ', o, '%')

