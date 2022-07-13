#!/usr/bin/env python3

import sys
import time
import gpiozero

if __name__=="__main__":
	k_p = sys.argv[1]
	k_i = sys.argv[2]
	k_d = sys.argv[3]

	pend_ang_before = 0.0

	time_before = time.perf_counter()

	pend_encoder = gpiozero.MCP3208(channel=0)
	motorL = gpiozero.Motor(forward=23, backward=24)
	motorR = gpiozero.Motor(forward=27, backward=22)

	while True:
		time.sleep(0.01)

		pend_ang = pend_encoder.value * (3.14159 / 5.0) - 3.14159

		time_now = time.perf_counter()
		dt = time_now - time_before
		err_p = pend_ang
		err_i = (pend_ang + pend_ang_before) * dt / 2.0
		err_d = (pend_ang - pend_ang_before) / dt

		o = k_p * err_p + k_i * err_i + k_d * err_d

		if o > 1.0:
			o = 1.0
		if o < -1.0:
			o = -1.0

		if o >= 0.0:
			motorL.forward(o)
			motorR.forward(o)
		else:
			motorL.backward(-o)
			motorR.backward(-o)



