#!/usr/bin/env python3

import rospy
import gpiozero
import time
from std_msgs.msg import Float32

#rad sec m kg V
class MyMotor():
	DELTA_T = 0.01
	target_angv = 0.0
	STEP_2_RAD = 3.14159 / 40.0
	
	a_volt = 9.0
	a_torque = -400.0
	a_angv = -2.0 / (125.0 * 3.14159)
	
	l = 4 #length of latest_angs
	latest_delta_angs = [0.0] * 4 #0:newest ang , 1:1 loop old ...
	latest_step = 0
	s = 1.0
	
	o = 0.0

	time_stamp = 0.0

	def __init__(self, forward_gpiono, backward_gpiono, encoder_a_gpiono, encoder_b_gpiono):
		self.motor = gpiozero.Motor(forward=forward_gpiono, backward=backward_gpiono)
		self.encoder = gpiozero.RotaryEncoder(a=encoder_a_gpiono, b=encoder_b_gpiono, max_steps=40, wrap=True)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_T), self.timer_func)
#		self.sub = rospy.Subscriber('pos_v', Float32, self.update_taget)
		self.sub = rospy.Subscriber('tire_angv', Float32, self.update_target)

		self.s = 1.0 - (0.5 ** self.l)


	def timer_func(self, event):
		if time.now() - self.time_stamp > 0.05:
			#calc angular velocity
			for i in range(1, self.l):
				self.latest_delta_angs[self.l - i] = self.latest_delta_angs[self.l - i - 1]
			step_before = self.latest_step
			self.latest_step = self.encoder.steps
			delta_step = (self.latest_step - step_before + 81) % 81
			if delta_step >= 41:
				delta_step -= 81
			self.latest_delta_angs[0] = float(delta_step) * self.STEP_2_RAD

			angv = 0.0
			p = 1.0
			d = 0.0
			for i in range(self.l):
				p *= 0.5
				d += self.latest_delta_angs[i]
				angv += p * d / (float(i + 1) * self.DELTA_T)
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


	def update_target(self, msg):
		self.time_stamp = time.now()

		self.target_angv = msg.data
		self.o = (- self.target_angv * self.a_angv) / self.a_volt
		if self.o >= 1.0:
			self.motor.forward(1.0)
		elif self.o >= 0.0:
			self.motor.forward(self.o)
		elif self.o <= -1.0:
			self.motor.backward(1.0)
		else:
			self.motor.backward(-self.o)


def main():
	try:
		rospy.init_node('motor_test', anonymous=True)
		f = rospy.get_param('/motor_test/forward_gpio')
		b = rospy.get_param('/motor_test/backward_gpio')
		A = rospy.get_param('/motor_test/encoder_a_gpio')
		B = rospy.get_param('/motor_test/encoder_b_gpio')
		mymotor = MyMotor(f, b, A, B)
		rospy.spin()
	except rospy.ROSInterruptException: pass


if __name__ == "__main__":
	main()

