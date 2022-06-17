#!/usr/bin/env python3

import rospy
import gpiozero
import time

#rad
class MyMotor():
	DELTA_T = 0.002
	target_angv = 0.0
	STEP_2_RAD = 3.14159 / 40.0
	angv = 0.0
	
	err_angv = 0.0
	err_angv_d = 0.0
	err_angv_i = 0.0

	k_p = 1.0
	k_d = 1.0
	k_i = 1.0

	l = 5 #length of latest_angs
	latest_angs = [0.0] * 5 #0:newest ang , 1:1 loop old ...
	s = 1.0

	def __init__(self, forward_gpiono, backward_gpiono, encoder_a_gpiono, encoder_b_gpiono):
		self.motor = gpiozero.Motor(forward_gpiono, backward_gpiono)
		self.encoder = gpiozero.RotaryEncoder(a=encoder_a_gpiono, b=encoder_b_gpiono, max_steps=400)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_T), self.timer_func)

		self.k_p = rospy.get_param('/motor_test/k_p')
		self.k_i = rospy.get_param('/motor_test/k_i')
		self.k_d = rospy.get_param('/motor_test/k_d')
		self.s = 1.0 - (0.5 ** self.l)


	def timer_func(self, event):
		#calc angv
		for i in range(1, self.l):
			self.latest_angs[self.l - i] = self.latest_angs[self.l - i - 1]
		self.latest_angs[0] = float(self.encoder.steps) * self.STEP_2_RAD

		self.angv = 0.0
		p = 1.0
		for i in range(1, self.l):
			p *= 0.5
			self.angv += p * (self.latest_angs[0] - self.latest_angs[i]) / (float(i) * self.DELTA_T)
		self.angv /= self.s
		
		#calc error and its d&i
		e_av_b = self.err_angv
		self.err_angv = self.target_angv - self.angv
		
		self.err_angv_d = (self.err_angv - e_av_b) / self.DELTA_T
		self.err_angv_i += (self.err_angv + e_av_b) * self.DELTA_T / 2.0

		#output
		o = self.err_angv * self.k_p + self.err_angv_d * self.k_d + self.err_angv_i * self.k_i
		if o > 1.0:
			self.motor.forward(1.0)
		elif o >= 0.0:
			self.motor.forward(o)
		elif o < -1.0:
			self.motor.backward(1.0)
		else:
			self.motor.backward(-o)


	def rotate(self, rad_per_sec):
		self.target_angv = rad_per_sec
		self.err_angv = 0.0
		self.err_ang = 0.0


def main():
	try:
		rospy.init_node('motor_test', anonymous=True)
		mymotor = MyMotor(23, 24, 5, 6)

		print('f slow')
		mymotor.rotate(3.14159)
		time.sleep(1.0)
		print('f fast')
		mymotor.rotate(3.14159 * 2.0)
		time.sleep(1.0)
		print('f slow')
		mymotor.rotate(3.14159)
		time.sleep(1.0)
		print('stop')
		mymotor.rotate(0.0)
		time.sleep(1.0)
		print('b slow')
		mymotor.rotate(-3.14159)
		time.sleep(1.0)
		print('b fast')
		mymotor.rotate(-3.14159 * 2.0)
		time.sleep(1.0)
		print('b slow')
		mymotor.rotate(-3.14159)
		time.sleep(1.0)
		print('stop')
		mymotor.rotate(0.0)
		time.sleep(1.0)
		print('finish')
		
	except rospy.ROSInterruptException: pass


if __name__ == "__main__":
	main()

