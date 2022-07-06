#!/usr/bin/env python3

import math
import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray

GRAVITY = 9.81

class ControllerNode():
	DELTA_TIME = 0.02
	
	m = 0.075
	M = 0.74
	I = 0.004
	r = 0.12

	pendulum_ang = 0.0
	pendulum_angv = 0.0
	car_pos = 0.0
	car_vel = 0.0

	eigenvals = [0.0] * 4

	k_ang = 0.0
	k_angv = 0.0
	k_pos = 0.0
	k_posv = 0.0


	def __init__(self):
		self.sub_ang = rospy.Subscriber('state', Float32MultiArray, self.on_update_state)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_TIME), self.update_pos_v)
		self.pub_vel = rospy.Publisher('target_car_vel', Float32, queue_size=10)
		self.eigenvals[0] = rospy.get_param("/controller/eigenval0")
		self.eigenvals[1] = rospy.get_param("/controller/eigenval1")
		self.eigenvals[2] = rospy.get_param("/controller/eigenval2")
		self.eigenvals[3] = rospy.get_param("/controller/eigenval3")
		if self.eigenvals[0] < 0.0 and self.eigenvals[1] < 0.0 and self.eigenvals[2] < 0.0 and self.eigenvals[3] < 0.0:
			self.calc_k()

		
	def calc_k(self):
		A = np.zeros((4,4))
		b = np.zeros((4,1))
		for i in range(4):
			A[i][0] = self.m * GRAVITY / self.r
			A[i][1] = (self.I / (self.r ** 2) - self.m) * (self.eigenvals[i] ** 3) + self.m * GRAVITY * self.eigenvals[i] / self.r
			A[i][2] = self.m * (self.eigenvals[i] ** 2)
			A[i][3] = self.m * (self.eigenvals[i] ** 3)
			b[i][0] = ((self.M + self.m) * (self.I / (self.r ** 2) - self.m) + self.m ** 2) * (self.eigenvals[i] ** 4)
			b[i][0] += (self.m * (self.m + self.M) * GRAVITY * (self.eigenvals[i] ** 2)) / self.r
		try:
			ans = np.linalg.solve(A, b)
			self.k_pos = ans[0][0]
			self.k_posv = ans[1][0]
			self.k_ang = ans[2][0]
			self.k_angv = ans[3][0]
		except np.linalg.LinAlgError: pass


	def on_update_state(self, msg):
		self.car_pos = msg.data[0]
		self.car_vel = msg.data[1]
		self.pendulum_ang = msg.data[2]
		self.pendulum_angv = msg.data[3]


	def update_pos_v(self, event):
		if not rospy.is_shutdown():
			f = self.k_pos * self.car_pos
			f += self.k_posv * self.car_vel
			f += self.k_ang * self.r * self.pendulum_ang
			f += self.k_angv * self.r * self.pendulum_angv

			temp = (self.m * self.m * self.r * self.r) / (self.I - self.m * self.r * self.r)
			pos_a = temp * GRAVITY + self.m * self.r * (self.pendulum_angv ** 2)
			pos_a *= math.sin(self.pendulum_angv)
			pos_a += f
			pos_a /= temp
			new_car_vel = self.car_vel + pos_a * self.DELTA_TIME
			self.pub_vel.publish(new_car_vel)


def main():
	try:
		rospy.init_node('controller', anonymous=True)
		node = ControllerNode()
		rospy.spin()
	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
