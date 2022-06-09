#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class ControllerNode():
	DELTA_TIME = 0.02

	ang = 0.0
	ang_v = 0.0
	pos_v = 0.0

	k_ang = 0.0
	k_ang_v = 0.0
	k_pos_v = 0.0


	def __init__(self):
		self.sub_ang = rospy.Subscriber('ang', Float32, self.on_ang_update)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_TIME), self.update_pos_v)
		self.pub_pos_v = rospy.Publisher('pos_v', Float32, queue_size=10)


	def on_ang_update(self, msg):
		self.ang = msg.data


	def update_pos_v(self, event):
		if not rospy.is_shutdown():
			self.pub_pos_v.publish(self.pos_v)


def main():
	try:
		rospy.init_node('controller', anonymous=True)
		node = ControllerNode()
		rospy.spin()
	except rospy.ROSInterruptExceprion: pass

if __name__ == '__main__':
	main()
