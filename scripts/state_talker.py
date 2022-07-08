#!/usr/bin/env python3

#import spidev
import time
import gpiozero
import rospy
from std_msgs.msg import Float32MultiArray, Bool

ENCODER_VALUE_2_RAD = 3.14159 * 2.0
MOTOR_ENCODER_STEP_2_RAD = 3.14159 / 40.0
MOTOR_ENCODER_MAX_STEP = 40
TIRE_RADIUS = 0.025
FAIL_ANG_RAD = 3.14159 / 3.0
			

def check_fail(ang):
	return False
#	if ang > FAIL_ANG_RAD or ang < -FAIL_ANG_RAD:
#		return True
#	else:
#		return False


def main():
#	spi = spidev.SpiDev()
#	spi.open(0,0)
#	spi.max_speed_hz=10000
	pendulum_encoder = gpiozero.MCP3208(channel=0)
	encoder_a = rospy.get_param('/state_talker/motorL_encoder_a')
	encoder_b = rospy.get_param('/state_talker/motorL_encoder_b')
	motorL_encoder = gpiozero.RotaryEncoder(a=encoder_a, b=encoder_b, max_steps=MOTOR_ENCODER_MAX_STEP, wrap=True)
	encoder_a = rospy.get_param('/state_talker/motorR_encoder_a')
	encoder_b = rospy.get_param('/state_talker/motorR_encoder_b')
	motorR_encoder = gpiozero.RotaryEncoder(a=encoder_a, b=encoder_b, max_steps=MOTOR_ENCODER_MAX_STEP, wrap=True)
	
	l = 3
	pendulum_ang_before = 0.0
	pendulum_latest_delta_angs = [0.0] * 3
	motorR_ang_before = 0.0
	motorR_latest_delta_angs = [0.0] * 3
	motorL_ang_before = 0.0
	motorL_latest_delta_angs = [0.0] * 3
	time_before = time.perf_counter
	latest_delta_times = [1.0/100.0] * 3
	s = 1.0 - (0.5 ** l)

	try:
		rospy.init_node('state_talker', anonymous=True)
		#state=[car's pos  ,  car's vel  ,  pendulum's ang  ,  pendulum's angv]
		pub_state = rospy.Publisher('state', Float32MultiArray, queue_size=10)
		
		pub_motor = rospy.Publisher('tire_angv', Float32MultiArray, queue_size=10)
		pub_fail = rospy.Publisher('failed', Bool, queue_size=10)
		r = rospy.Rate(100)
		delta_t = 1.0 / 100.0
		

		while not rospy.is_shutdown():
			pendulum_ang = pendulum_encoder.value * ENCODER_VALUE_2_RAD - 3.14159
			if check_fail(pendulum_ang):
#				print('check fail')
				pub_fail.publish(True)
#				break
			else:
				for i in range(1, l):
					pendulum_latest_delta_angs[i] = pendulum_latest_delta_angs[i - 1]
					motorR_latest_delta_angs[i] = motorR_latest_delta_angs[i - 1]
					motorL_latest_delta_angs[i] = motorL_latest_delta_angs[i - 1]
					latest_delta_times[i] = latest_delta_times[i - 1]
				motorL_ang = float(motorL_encoder.steps) * MOTOR_ENCODER_STEP_2_RAD
				motorR_ang = float(motorR_encoder.steps) * MOTOR_ENCODER_STEP_2_RAD
				time_now = time.perf_counter

				pendulum_latest_delta_angs[0] = pendulum_ang - pendulum_ang_before
				motorL_latest_delta_angs[0] = motorL_ang - motorL_ang_before
				if motorL_latest_delta_angs[0] > 3.14159:
					motorL_latest_delta_angs[0] -= 2.0 * 3.14159
				if motorL_latest_delta_angs[0] < -3.14159:
					motorL_latest_delta_angs[0] += 2.0 * 3.14159
				motorR_latest_delta_angs[0] = motorR_ang - motorR_ang_before
				if motorR_latest_delta_angs[0] > 3.14159:
					motorR_latest_delta_angs[0] -= 2.0 * 3.14159
				if motorR_latest_delta_angs[0] < -3.14159:
					motorR_latest_delta_angs[0] += 2.0 * 3.14159
				latest_delta_times[0] = time_now - time_before

				car_pos = (motorL_ang + motorR_ang) * TIRE_RADIUS / 2.0
				
				pendulum_angv = 0.0
				motorL_angv = 0.0
				motorR_angv = 0.0
				d_p = 0.0
				d_R = 0.0
				d_L = 0.0
				d_t = 0.0
				p = 1.0
				for i in range(l):
					p *= 0.5
					d_p += pendulum_latest_delta_angs[i]
					d_R += motorR_latest_delta_angs[i]
					d_L += motorL_latest_delta_angs[i]
					d_t += latest_delta_times[i]
					pendulum_angv += p * d_p / (float(i + 1) * d_t)
					motorL_angv += p * d_L / (float(i + 1) * d_t)
					motorR_angv += p * d_R / (float(i + 1) * d_t)

				car_vel = (motorL_angv + motorR_angv) * TIRE_RADIUS / 2.0

				arr = [pendulum_ang, pendulum_angv, car_pos, car_vel]
				pub_state.publish(Float32MultiArray(data=arr))
				
				pub_motor.publish(Float32MultiArray(data=[motorL_angv, motorR_angv]))
				
				pendulum_ang_before = pendulum_ang
				motorL_ang_before = motorL_ang
				motorR_ang_before = motorR_ang
				time_before = time_now
			r.sleep()

	except rospy.ROSInterruptException: pass


if __name__ == '__main__':
	main()
