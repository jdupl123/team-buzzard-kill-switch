""" State estimator """
""" Takes GPS strings and IMU data to output current state in coords, speed and heading """

class StateEstimator:
	state = {}

	def __init__(self):
		self.state['velocity'] = 0
		self.state['x'] = ()
		self.state['y'] = ()
		self.state['bearing'] = 0
		self.state['fixed'] = False

	def state_estimator(self, gps_string):
		if gps_string.startswith("$GPGGA"):
			parts = gps_string.split(',')
			if len(parts) != 15:
				print("string too short...")
				return # GPS message mangled, return old state --> TODO: predict new state from old state

			fixed = parts[6]
			num_satellites = parts[7]
			if int(fixed) > 0 and int(num_satellites) > 6:
				self.state['fixed'] = True
				print("updated state to fixed")
				# TODO: predict new state from old state?
				return
			else:
				print("state still unfixed")
				self.state['fixed'] = False
				return

		elif gps_string.startswith("$GPRMC") and self.state['fixed']:
		# TODO: fuzzy string matching in case the string is messy? gps_string.split(',')[0].lower()
			parts = gps_string.split(',')

			if len(parts) != 13:
				return

			lat = float(parts[3]) # utm -> just assume south and east lol
			lon = float(parts[5])
			speed = round(float(parts[7])*1.852, 2) # convert knots to km/hr and round cos why have so much precision lol
			bearing = float(parts[8]) # will be between 0 and 360

			self.state['y'] = lat
			self.state['x'] = lon
			self.state['velocity'] = speed
			self.state['bearing'] = bearing

			return

		else:
			print("incorrect string type")
			return


