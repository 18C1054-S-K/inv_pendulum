#!/usr/bin/env python3

import gpiozero
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool

#rad
class MyMotor():
	is_left = True
	
	DELTA_T = 0.01
	target_angv = 0.0
	angv = 0.0
	
	err_angv = 0.0
	err_angv_d = 0.0
	err_angv_i = 0.0

	k_p = 1.0
	k_d = 1.0
	k_i = 1.0


	def __init__(self, is_Left):
		if is_Left == True:
			self.is_left = True
		else:
			self.is_left = False
		f = 3
		b = 4
		if self.is_left:
			f = rospy.get_param('/tire_rotater/motorL_for')
			b = rospy.get_param('/tire_rotater/motorL_back')
		else:
			f = rospy.get_param('/tire_rotater/motorR_for')
			b = rospy.get_param('/tire_rotater/motorR_back')
		self.k_p = rospy.get_param('/tire_rotater/k_p')
		self.k_i = rospy.get_param('/tire_rotater/k_i')
		self.k_d = rospy.get_param('/tire_rotater/k_d')
		self.motor = gpiozero.Motor(forward=f, backward=b)
		self.sub_tire_angv = rospy.Subscriber('tire_angv', Float32MultiArray, self.update_tire_angv)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_T), self.timer_func)


	def update_tire_angv(self, msg):
		if self.is_left:
			self.angv = msg.data[0]
		else:
			self.angv = msg.data[1]


	def timer_func(self, event):
		#calc error and its d&i
		e_av_b = self.err_angv
		self.err_angv = self.target_angv - self.angv
		
		self.err_angv_d = (self.err_angv - e_av_b) / self.DELTA_T
		self.err_angv_i += (self.err_angv + e_av_b) * self.DELTA_T / 2.0

		#output
		o = self.err_angv * self.k_p + self.err_angv_d * self.k_d + self.err_angv_i * self.k_i
		if not self.is_left:
			o *= -1.0
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
		self.err_angv_i = 0.0
		
		
	def stop(self):
		self.motor.stop()


class TireRotaterNode:
	POS_V_2_RAD_PER_SEC = 0.0
	MAX_RAD_PER_SEC = 5.0
	TIRE_RADIUS = 0.03

	rad_per_sec = 0.0
	
	controll_failed = False

	def __init__(self):
		self.motorL = MyMotor(is_Left=True)
		self.motorR = MyMotor(is_Left=False)
		self.sub_target = rospy.Subscriber('target_car_vel', Float32, self.on_update_target)
		self.sub_fail = rospy.Subscriber('failed', Bool, self.on_controll_failed)


	def on_update_target(self, msg):
		if not self.controll_failed:
			self.motorL.rotate(msg.data / self.TIRE_RADIUS)
			self.motorR.rotate(msg.data / self.TIRE_RADIUS)


	def on_controll_failed(self, msg):
		if msg.data == True:
			self.controll_failed = True
			self.motorL.stop()
			self.motorR.stop()


def main():
	try:
		rospy.init_node('tire_rotater', anonymous=True)
		node = TireRotaterNode()
		rospy.spin()
	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
