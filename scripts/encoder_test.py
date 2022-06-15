#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

ang=0.0
test_b = True

def update_ang(msg):
	if test_b:
		print('update_ang')
		test_b=False
	ang = msg.data


def print_ang(event):
	test_b=True
	print(ang)


def main():
	print('ang')
	try:
		rospy.init_node('encoder_test', anonymous=True)
		rospy.Subscriber('ang', Float32, update_ang)
		rospy.Timer(rospy.Duration(1.2), print_ang)
		rospy.spin()
	except rospy.ROSInterruptException: pass


if __name__ == "__main__":
	main()
