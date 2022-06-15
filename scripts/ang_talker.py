#!/usr/bin/env python3

import gpiozero
import rospy
from std_msgs.msg import Float32, Bool

ENCODER_VALUE_2_RAD = 3.14159
FAIL_ANG_RAD = 3.14159 / 3.0
			

def check_fail(ang):
	if ang > FAIL_ANG_RAD or ang < -FAIL_ANG_RAD:
		return True
	else:
		return False


def main():
	encoder = gpiozero.MCP3208(channel=0)

	try:
		rospy.init_node('ang_talker', anonymous=True)
		pub_ang = rospy.Publisher('ang', Float32, queue_size=10)
		pub_fail = rospy.Publisher('failed', Bool, queue_size=10)

		while not rospy.is_shutdown():
			ang = encoder.value * ENCODER_VALUE_2_RAD
			if check_fail(ang):
				pub_fail.publish(True)
				break
			else:
				pub_ang.publish(ang)

	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
