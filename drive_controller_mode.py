import matplotlib.pyplot as plt
import math
import numpy as np

class DriveController:

	def __init__(self):
		# car parameters - should measure max_turn
		self.L = 2.4
		self.max_turn = 55.0

		# theta control parameters and error values
		self.Ktheta_p = 0.1
		self.Ktheta_i = 0.001
		self.Ktheta_d = 0.01
		self.theta_i_err = 0
		self.theta_d_err = 0
		self.theta_d_last = 0
		self.Kv_i = 0.001
		self.Kv_d = 0.01
		self.Kv_p = 0.1
		self.vel_i_err = 0
		self.vel_d_err = 0
		self.vel_d_last = 0
	
	def run_drive_controller(self, desired_state, actual_state):
		"""
			desired_state and actual_state both contain:
				velocity,
				position_x,
				position_y,
				bearing

		"""
		vel_err = desired_state['velocity'] - actual_state['velocity'] # decides whether to accelerate or deccelerate
		self.vel_i_err += vel_err
		self.vel_d_err = (self.vel_d_last - vel_err)
		self.vel_d_last = vel_err
		# again do some mapping to accelerate max, mid, hold, deccel mid, max (i.e. stop)

		if actual_state['bearing'] > 270 and desired_state['bearing'] < 90:
			# turn forward over the 360 degree line
			theta_dot = (360 - actual_state['bearing'] + desired_state['bearing']) 
			self.theta_i_err += theta_dot # calculate the integral error
			self.theta_d_err = (theta_dot - self.vel_d_last) # calculate the derivative error
			self.theta_d_last = theta_dot # update the derivative error
		elif desired_state['bearing'] > 270 and actual_state['bearing'] < 90:
			# turn back over the 360 degree line
			theta_dot = (-(360 - desired_state['bearing'] + actual_state['bearing']))
			self.theta_i_err += theta_dot
			self.theta_d_err = (theta_dot - self.vel_d_last)
			self.theta_d_last = theta_dot
		else:
			theta_dot = (desired_state['bearing'] - actual_state['bearing'])
			self.theta_i_err += theta_dot
			self.theta_d_err = (theta_dot - self.vel_d_last)
			self.theta_d_last = theta_dot
		
		# cap the integral error to 5 degrees or -5 degrees so it doesn't blow up
		if self.theta_i_err > 5:
			self.theta_i_err = 5
		elif self.theta_i_err < -5:
			self.theta_i_err = -5 

		# theta_dot = v / L tan(steering_angle)
		# steering_angle = arctan(L * theta_dot / v)
		print(theta_dot)
		new_des_theta = self.Ktheta_p * theta_dot # + self.Ktheta_i * self.theta_i_err + self.Ktheta_d 
		if actual_state['velocity'] == 0:
			actual_state['velocity'] = 1
		desired_steering_angle = math.atan((self.L * new_des_theta) / actual_state['velocity']) * (180/np.pi)
		print(desired_steering_angle)
		# some thresholding here to stop oversteering or driving the steering too far ->
		if desired_steering_angle > self.max_turn:
			desired_steering_angle = self.max_turn
		elif desired_steering_angle < -self.max_turn:
			desired_steering_angle = -self.max_turn
		print(desired_steering_angle)
		# map desired_steering_angle to a pot voltage or whatever it is lol
		# need potentiometer value on straight, max left and max right steering
		# 512 = straight, 0 = full left, 1024 = full right
		print('checking_max_turn: ', self.max_turn)
		steer_output = ((desired_steering_angle + self.max_turn)/(2*self.max_turn)) * 1024.0
		print(steer_output)
		# map vel_err to a throttle position
		#################################################################
		# WARNING: This code needs fixing -> mapping may not be correct #
		#################################################################
		if vel_err < -20:
			# rotate servo position -> towards 0 and extend actuator to apply brake
			# change in force applyed to the car (either through accel or braking)
			desired_vel_out = self.Kv_p * vel_err # + Kv_d * vel_d_err + Kv_i * vel_i_err # map between 0 and 255
			if desired_vel_out < -40:
				desired_vel_out = -40

			accel_mapping = ((-1*desired_vel_out/40)) * 100 + 40 # so 0 -> 140 and 40 -> 40

			brake_control = 0.5 * accel_mapping # half the power of deccel with the throttle

		else:
			# rotate servo position -> towards 140
			# send braking signal as 0 (all the way back)
			# drive brake position forwards -> linear accelerator
			brake_control = 0 # no braking while driving #gottagofast

			desired_vel_out = self.Kv_p * vel_err # + Kv_d * vel_d_err + Kv_i * vel_i_err
			if desired_vel_out > 40:
				desired_vel_out = 40

			accel_mapping = round((1 - (desired_vel_out/40)) * 100 + 40) # so 0 -> 140 and 40 -> 40
		
		return (accel_mapping, brake_control, steer_output)


