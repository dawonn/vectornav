Header header

float64 time	# GPS time of week in seconds
uint16 week		# GPS week (week)
uint64 utcTime	# The current UTC time. The year is given as a signed byte year offset from the year 2000. E.g. 2013 as 13.
                # Fields:       year    month    day    hour    min    sec    ms
                # Byte offset:  0       1        2      3       4      5      6|7

# INS Status
# Name		Bit Offset	Format	Description
# Mode		0			2 bits	Indicates the current mode of the INS filter.
#								0 = Not tracking. Insufficient dynamic motion to estimate attitude.
#								1 = Sufficient dynamic motion, but solution not within performance specs.
#								2 = INS is tracking and operating within specifications.
# GpsFix	2			1 bit	Indicates whether the GPS has a proper fix
# Error 	3			4 bits	Sensor measurement error code
#								0 = No errors detected.
# 								Name			Bit Offset	Format	Description
#								Time Error		0			1 bit	High if INS filter loop exceeds 5 ms.
#								IMU Error		1			1 bit	High if IMU communication error is detected.
#								Mag/Pres Error	2			1 bit	High if Magnetometer or Pressure sensor error is detected.
#								GPS Error		3			1 bit	High if GPS communication error is detected.
#Reserved	7			9 bits	Reserved for future use.
uint16 insStatus

float32 yaw		# Yaw angle relative to true north. (degree)
float32 pitch	# Yaw angle relative to true north (degree)
float32 roll	# Pitch angle relative to horizon (degree)

float64 latitude	# INS solution position in geodetic latitude (degree)
float64 longitude	# INS solution position in geodetic longitude (degree)
float64 altitude	# Height above ellipsoid. (WGS84) (meter)

float32 nedVelX		# INS solution velocity in NED frame. (North) (m/s)
float32 nedVelY		# INS solution velocity in NED frame. (East) (m/s)
float32 nedVelZ		# INS solution velocity in NED frame. (Down) (m/s)

float32[3] attUncertainty	# Uncertainty in attitude estimate (yaw, pitch and roll in degrees)
float32 posUncertainty	# Uncertainty in position estimate (m)
float32 velUncertainty	# Uncertainty in velocity estimate (m/s)
