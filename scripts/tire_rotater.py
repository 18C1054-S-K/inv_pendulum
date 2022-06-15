#!/usr/bin/env python3

import gpiozero
import rospy
from std_msgs.msg import Float32, Bool

def TireRotaterNode():
	POS_V_2_RAD_PER_SEC = 0.0
	MAX_RAD_PER_SEC = 5.0

	rad_per_sec = 0.0
	
	controll_failed = False

	def __init__(self):
		self.motor = gppiozero.Motor(17, 18)
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
