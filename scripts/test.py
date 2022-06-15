#!/usr/bin/env python3

import rospy

if __name__ == "__main__":
	rospy.init_node('test', anonymous=True)
	p = rospy.get_param("/test/test_param")
	print('p = ', p)
	print('typr of p is ', type(p))
