import matplotlib.pyplot as plt
import math
SIM = True
K_theta = 0.5
L = 2.4

def run_drive_controller(desired_state, actual_state):
	"""
		desired_state and actual_state both contain:
			velocity,
			position_x,
			position_y,
			bearing

	"""
	accel = desired_state['velocity'] - actual_state['velocity'] # decides whether to accelerate or deccelerate
	# again do some mapping to accelerate max, mid, hold, deccel mid, max (i.e. stop)

	if actual_state['bearing'] > 270 and desired_state['bearing'] < 90:
		# turn forward over the 360 degree line
		theta_dot = K_theta * (360 - actual_state['bearing'] + desired_state['bearing']) 
	elif desired_state['bearing'] > 270 and actual_state['bearing'] < 90:
		# turn back over the 360 degree line
		theta_dot = K_theta * (-(360 - desired_state['bearing'] + actual_state['bearing']))
	else:
		theta_dot = K_theta * (desired_state['bearing'] - actual_state['bearing'])

	# some thresholding here to stop oversteering or driving the steering too far ->
	# if theta_dot > 60: ??
	# 	theta_dot = 60

	# theta_dot = v / L tan(steering_angle)
	# steering_angle = arctan(L * theta_dot / v)
	desired_steering_angle = math.atan(L * theta_dot / actual_state['velocity'])

	# map accel to a throttle position
	if accel < 0 and accel > -30:
		# rotate servo position -> towards 0

	elif accel < -30:
		# rotate servo position -> towards 140
		# drive brake position forwards -> linear accelerator

	elif accel >= 0 and accel < 15:
		# rotate servo position -> towards 40 (to ... 30%?)

	elif accel >= 15:
		# drive throttle position forwards to half as a starting point lol

	# map desired_steering_angle to a pot voltage or whatever it is lol
	# need potentiometer value on straight, max left and max right steering
	if theta_dot < -5:
		# turn left

	elif theta_dot > 5: 
		# turn right

	else:
		# hold steering angle --> how to return this?





