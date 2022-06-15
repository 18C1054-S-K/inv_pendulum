#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32

GRAVITY = 9.81

class ControllerNode():
	DELTA_TIME = 0.01
	
	m = 0.1
	M = 0.8
	I = 0.1
	r = 0.12

	ang = 0.0
	ang_v = 0.0
	pos_v = 0.0

	eigenvals = [0.0, 0.0, 0.0]

	k_ang = 0.0
	k_ang_v = 0.0
	k_pos_v = 0.0

	f_2 = 0.0
	f_3 = 0.0
	f_4 = 0.0

	def __init__(self):
		self.sub_ang = rospy.Subscriber('ang', Float32, self.on_ang_update)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_TIME), self.update_pos_v)
		self.pub_pos_v = rospy.Publisher('pos_v', Float32, queue_size=10)
		self.eigenvals[0] = rospy.get_param("/controller/eigenval0")
		self.eigenvals[1] = rospy.get_param("/controller/eigenval1")
		self.eigenvals[2] = rospy.get_param("/controller/eigenval2")
		if self.eigenvals[0] < 0.0 and self.eigenvals[1] < 0.0 and self.eigenvals[2] < 0.0:
			self.calc_k_and_f()

		
	def calc_k_and_f(self):
		A = np.zeros(3,3)
		b = np.zeros(3,1)
		for i in range(3):
			A[i][0] = (self.I / (self.r ** 2) - self.m) * (self.eigenvals[i] ** 2) + self.m * GRAVITY/ self.r
			A[i][1] = self.m * self.eigenvals[i]
			A[i][2] = self.m * (self.eigenvals[i] ** 2)
			b[i][0] = ((self.M + self.m) * (self.I / (self.r ** 2) - self.m) + self.m ** 2) * (self.eigenvals[i] ** 3)
			b[i][0] += (self.m * (self.m + self.M) * GRAVITY * self.eigenvals[i]) / self.r
		try:
			ans = np.linalg.solve(A, b)
			self.k_pos_v = ans[0][0]
			self.k_ang = ans[1][0]
			self.k_ang_v = ans[2][0]

			temp = (self.m + self.M) * (self.I / (self.r ** 2) - m) + self.m ** 2
			temp = 1.0 / temp
			self.f_2 = self.k_pos_v * (self.I / (self.r ** 2) - m) * temp
			self.f_3 = (self.m ** 2) * GRAVITY / self.r + (self.I / (self.r ** 2) - m) * self.k_ang
			self.f_3 *= temp
			self.f_4 = temp * (self.I / (self.r ** 2) - m) * self.k_ang_v
		except np.linalg.LinAlgError: pass


	def on_ang_update(self, msg):
		self.ang = msg.data


	def update_pos_v(self, event):
		if not rospy.is_shutdown():
			pos_a = self.f_2 * self.pos_v + self.f_3 * self.ang + self.f_4 * self.ang_v
			self.pos_v = pos_a * self.DELTA_TIME
			self.pub_pos_v.publish(self.pos_v)


def main():
	try:
		rospy.init_node('controller', anonymous=True)
		node = ControllerNode()
		rospy.spin()
	except rospy.ROSInterruptExceprion: pass


if __name__ == '__main__':
	main()
