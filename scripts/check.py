#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray


class CheckNode:
	pendulum_ang = 0.0
	pendulum_angv = 0.0
	car_pos = 0.0
	car_vel = 0.0
	
	motorL_angv = 0.0
	motorR_angv = 0.0


	def __init__(self):
		rospy.init_node('check', anonymous=True)
		self.sub_state = rospy.Subscriber('state', Float32MultiArray, self.OnUpdateState)
		self.sub_motor = rospy.Subscriber('motor', Float32MultiArray, self.OnUpdateMotor) #debug
		self.timer_disp = rospy.Timer(rospy.Duration(1.2), self.Disp)


	def OnUpdateState(self, msg):
		self.pendulum_ang = msg.data[0]
		self.pendulum_angv = msg.data[1]
		self.car_pos = msg.data[2]
		self.car_vel = msg.data[3]


	def OnUpdateMotor(self, msg): #debug
		self.motorL_angv = msg.data[0]
		self.motorR_angv = msg.data[1]

	def Disp(self, event):
		print('')
		print('pnd ang  : ', self.pendulum_ang)
		print('pnd angv : ', self.pendulum_angv)
		print('car pos  : ', self.car_pos)
		print('car vel  : ', self.car_vel)
		print('')
		print('    motorL_angv : ', self.motorL_angv)
		print('    motorR_angv : ', self.motorR_angv)


if __name__=="__main__":
	node = CheckNode()

