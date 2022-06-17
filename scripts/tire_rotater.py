#!/usr/bin/env python3

import gpiozero
import rospy
from std_msgs.msg import Float32, Bool

#rad
class MyMotor():
	DELTA_T = 0.003
	target_angv = 0.0
	STEP_2_RAD = 3.14159 / 100.0
	angv = 0.0
	
	err_angv = 0.0
	err_angv_d = 0.0
	err_angv_i = 0.0

	k_p = 1.0
	k_d = 1.0
	k_i = 1.0

	l = 4 #length of latest_angs
	latest_angs = [0.0] #0:newest ang , 1:1 loop old ...
	s = 1.0

	def __init__(self, forward_gpiono, backward_gpiono, encoder_a_gpiono, encoder_b_gpiono):
		self.motor = gpiozero.Motor(forward_gpiono, backward_gpiono)
		self.encoder = gpiozero.RotaryEncoder(a=encoder_a_gpiono, b=encoder_b_gpiono, max_steps=100)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_T), self.timer_func)

		self.latest_angs = [0.0] * self.l
		self.s = 1.0 - (0.5 ** self.l)


	def timer_func(self, event):
		#calc angv
		for i in range(1, self.l):
			latest_angs[self.l - i] = self.latest_angs[self.l - i - 1]
		self.latest_angs[0] = self.encoder.steps * self.STEP_2_RAD

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
		self.target_rps = rad_per_sec
		self.err_angv = 0.0
		self.err_ang = 0.0


class TireRotaterNode():
	POS_V_2_RAD_PER_SEC = 0.0
	MAX_RAD_PER_SEC = 5.0

	rad_per_sec = 0.0
	
	controll_failed = False

	def __init__(self):
		self.motor = gpiozero.Motor(17, 18)
		self.sub_pos_v('pos_v', Float32, self.on_update_pos_v)
		self.sub_fail('failed', Bool self.on_controll_failed)


	def on_update_pos_v(self, msg):
		if not controll_failed:
			self.rad_per_sec = msg.data * POS_V_2_RAD_PER_SE
			o = self.rad_per_sec / self.MAX_RAD_PER_SEC
			if o >= 1.0:
				self.motor.forward(1.0)
			elif o >=0.0:
				self.motor.forward(o)
			if o < -1.0:
				self.motor.backward(1.0)
			elif o < 0.0:
				self.motor.backward(-o)


	def on_controll_failed(self, msg):
		if msg.data == True:
			self.controll_failed = True
			delta_t = 1.0 / 30.0
			r = rospy.Rate(30)
			while self.rad_per_sec > 0.001 or self.rad_per_sec < -0.001:
				self.rad_per_sec = - self.rad_per_sec * dalta_t
				if self.rad_per_sec < 0.0:
					self.motor.backward(- self.rad_per_sec / self.MAX_RAD_PER_SEC)
				else:
					self.motor.forward(self.rad_per_sec / self.MAX_RAD_PER_SEC)
				r.sleep()
			self.motor.stop()


def main():
	try:
		rospy.init_node('tire_rotater', anonymous=True)
		node = TireControllerNode()
		rospy.spin()
	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
