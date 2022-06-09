#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class AngTalkerNode():
	VOLT_2_RAD = 3.14159 / 2.5
	FAIL_ANG_RAD = 3.14159 / 3.0

	volt = 0.0


	def __init__(self):
		self.pub_ang = rospy.Publisher('ang', Float32, queue_size=10)
		self.pub_fail = rospy.Publisher('failed', Bool, queue_size=10)


	def update(self):
		if not rospy.is_shutdown():
			ang = self.VOLT_2_RAD * self.volt
#			if self.check_fail(ang):
#				self.pub_fail.publish(True)
#			else:
			self.pub_ang.publish(ang)
			

	def check_fail(self, ang):
		if ang > FAIL_ANG_RAD or ang < -FAIL_ANG_RAD:
			return True
		else:
			return False


def main():
	try:
		rospy.init_node('ang_talker', anonymous=True)
		node = AngTalkerNode()
		rospy.spin()
	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
