""" State estimator """
""" Takes GPS strings and IMU data to output current state in coords, speed and heading """

def state_estimator(gps_string, last_state):
	if gps_string.startswith("$GPGGA"):
		parts = gps_string.split(',')
		if len(parts) != 15:
			return last_state # GPS message mangled, return old state --> TODO: predict new state from old state

		fixed = parts[6]
		num_satellites = parts[7]
		if fixed == 1 and num_satellites > 6:
			last_state['fixed'] = True
			# TODO: predict new state from old state?
			return last_state
		else:
			last_state['fixed'] = False
			return last_state

	if gps_string.startswith("$GPRMC"):
	# TODO: fuzzy string matching in case the string is messy? gps_string.split(',')[0].lower()
		parts = gps_string.split(',')

		if len(parts) != 12:
			return last_state

		lat = float(parts[3]) # utm -> just assume south and east lol
		lon = float(parts[5])
		speed = round(float(parts[7])*1.852, 2) # convert knots to km/s and round cos why have so much precision lol
		bearing = float(parts[8]) # will be between 0 and 360

		last_state['x'] = lat
		last_state['y'] = lon
		last_state['velocity'] = speed
		last_state['bearing'] = bearing

		return last_state


