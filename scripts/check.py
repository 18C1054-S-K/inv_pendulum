#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float32, Float32MultiArray


class StateTalkerCheckNode:
	pendulum_ang = 0.0
	pendulum_angv = 0.0
	car_pos = 0.0
	car_vel = 0.0
	
	motorL_angv = 0.0
	motorR_angv = 0.0


	def __init__(self):
		self.sub_state = rospy.Subscriber('state', Float32MultiArray, self.on_update_state)
		self.sub_motor = rospy.Subscriber('tire_angv', Float32MultiArray, self.on_update_motor) #debug
		self.timer_disp = rospy.Timer(rospy.Duration(1.2), self.disp)


	def on_update_state(self, msg):
		self.pendulum_ang = msg.data[0]
		self.pendulum_angv = msg.data[1]
		self.car_pos = msg.data[2]
		self.car_vel = msg.data[3]


	def on_update_motor(self, msg): #debug
		self.motorL_angv = msg.data[0]
		self.motorR_angv = msg.data[1]

	def disp(self, event):
		print('')
		print('pnd ang  : ', self.pendulum_ang)
		print('pnd angv : ', self.pendulum_angv)
		print('car pos  : ', self.car_pos)
		print('car vel  : ', self.car_vel)
		print('')
		print('    motorL_angv : ', self.motorL_angv)
		print('    motorR_angv : ', self.motorR_angv)


class MotorRotaterCheckNode:
	tireL_angv = 0.0
	tireR_angv = 0.0


	def __init__(self):
		TIRE_RADIUS = 0.03
		target_angv = rospy.get_param('/check/target_tire_angv')
		pub_target = rospy.Publisher('target_car_vel', Float32, queue_size=10)		
		time.sleep(0.5)
		pub_target.publish(target_angv * TIRE_RADIUS)
		print('target angv : ', target_angv)
		self.sub_tire_angv = rospy.Subscriber('tire_angv', Float32MultiArray, self.sub_tire_angv)
		self.timer = rospy.Timer(rospy.Duration(1.2), self.disp)
	
	
	def sub_tire_angv(self, msg):
		self.tireL_angv = msg.data[0]
		self.tireR_angv = msg.data[1]


	def disp(self, event):
		print('')
		print(' tireL angv : ', self.tireL_angv)
		print(' tireR angv : ', self.tireR_angv)


if __name__=="__main__":
	try:
		rospy.init_node('check', anonymous=True)
		node = MotorRotaterCheckNode()
		rospy.spin()
	except rospy.ROSInterruptException: pass

