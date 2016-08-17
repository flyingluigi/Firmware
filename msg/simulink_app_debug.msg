# This is similar to the mavlink message ATTITUDE, but for onboard use */
# @warning roll, pitch and yaw have always to be valid, the rotation matrix and quaternion are optional
float32[13] debug		# Debug data from simulink model: RateX,RateY,RateZ,VelX,VelY,VelXZ,fx,fy,fz,ux,uy,uz,mode
