#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool

def Motor:
	def __init__(self, pin_number=None):
		self.pig = pigpio.pi()
		self.pin_number = pin_number


	def rotate(self, rad_per_sec):
		if not self.pin_number == None:
			self.pig.set_pulsewidth(self.pin_number, )


def TireControllerNode():
	POS_V_2_RAD_PER_SEC = 0.0

	rad_per_sec = 0.0

	def __init__(self):
		self.motor = Motor(17)
		self.sub_pos_v('pos_v', Float32, self.on_update_pos_v)
#		self.sub_fail('failed', Bool self.on_controll_failed)


	def on_update_pos_v(self, msg):
		self.rad_per_sec = msg.data * POS_V_2_RAD_PER_SE
		self.motor.rotate(self.rad_per_sec)


	def on_controll_failed(self, msg):
		if msg == True:
			delta_t = 1.0 / 30.0
			r = rospy.Rate(30)
			while self.rad_per_sec > 0.0001 or self.rad_per_sec < -0.0001:
				self.rad_per_sec = - self.rad_per_sec * dalta_t
				self.motor.rotate(self.rad_per_sec)
				r.sleep()
			self.motor.rotate(0.0)


def main():
	pin_number = 17
	try:
		rospy.init_node('tire_controller', anonymous=True)
		node = TireControllerNode()
		rospy.spin(pin_number)
	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
