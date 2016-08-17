# This is similar to the mavlink message ATTITUDE, but for onboard use */
# @warning roll, pitch and yaw have always to be valid, the rotation matrix and quaternion are optional
bool arm 			# armin flag
uint16[8] pwm		# pwm in ms for esc output
