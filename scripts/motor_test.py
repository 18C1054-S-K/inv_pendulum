#!/usr/bin/env python3

import rospy
import gpiozero
import time

#rad sec m kg V
class MyMotor():
	DELTA_T = 0.002
	target_angv = 0.0
	STEP_2_RAD = 3.14159 / 40.0
	
	a_volt = 9.0
	a_torque = -400.0
	a_angv = -2.0 / (125.0 * 3.14159)
	
	l = 5 #length of latest_angs
	latest_delta_angs = [0.0] * 5 #0:newest ang , 1:1 loop old ...
	latest_step = 0
	s = 1.0
	
	o = 0.0

	def __init__(self, forward_gpiono, backward_gpiono, encoder_a_gpiono, encoder_b_gpiono):
		self.motor = gpiozero.Motor(forward_gpiono, backward_gpiono)
		self.encoder = gpiozero.RotaryEncoder(a=encoder_a_gpiono, b=encoder_b_gpiono, max_steps=40, wrap=True)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_T), self.timer_func)

		self.s = 1.0 - (0.5 ** self.l)


	def timer_func(self, event):
		#calc angular velocity
		for i in range(1, self.l):
			self.latest_delta_angs[self.l - i] = self.latest_delta_angs[self.l - i - 1]
		step_before = self.latest_step
		self.latest_step = self.encoder.steps
		delta_step = (self.latest_step - step_before + 81) % 81
		if delta_step > 41:
			delta_step -= 81
		self.latest_delta_angs[0] = float(delta_step) * self.STEP_2_RAD

		angv = 0.0
		p = 1.0
		d = 0.0
		for i in range(1, self.l):
			p *= 0.5
			d += self.latest_delta_angs[i]
			angv += p * d / (float(i) * self.DELTA_T)
		angv /= self.s
		
		#estimate torque
		tau = (- self.o * self.a_volt - angv * self.a_angv) / self.a_torque

		#fix output value
		self.o = (-tau * self.a_torque - self.target_angv * self.a_angv) / self.a_volt

		#output
		if self.o >= 1.0:
			self.motor.forward(1.0)
		elif self.o >= 0.0:
			self.motor.forward(self.o)
		elif self.o <= -1.0:
			self.motor.backward(1.0)
		else:
			self.motor.backward(-self.o)


	def rotate(self, rad_per_sec):
		self.target_angv = rad_per_sec


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

