# VectorNav Composite Data Packet
#
# Only the enabled fields will be populated with data and only the fields 
# you require for your application should be enabled to conserve bandwidth
# and increase the update rate.
#
# TODO[Dereck] Requires rosidl patches

# The timestamp is the ROS time at which data is received from the sensor
std_msgs/Header header

# Groups included in this packet
uint32	groups
uint32	BINARYGROUP_COMMON      = 0x01		
uint32	BINARYGROUP_TIME        = 0x02	
uint32	BINARYGROUP_IMU         = 0x04	
uint32	BINARYGROUP_GPS         = 0x08	
uint32	BINARYGROUP_ATTITUDE    = 0x10	
uint32	BINARYGROUP_INS         = 0x20		
uint32  BINARYGROUP_GPS2        = 0x40			

# Fields included in each group
uint32[]    group_field
uint32      COMMONGROUP_NONE			= 0x0000 
uint32      COMMONGROUP_TIMESTARTUP		= 0x0001 
uint32      COMMONGROUP_TIMEGPS			= 0x0002 
uint32      COMMONGROUP_TIMESYNCIN		= 0x0004 
uint32      COMMONGROUP_YAWPITCHROLL	= 0x0008 
uint32      COMMONGROUP_QUATERNION		= 0x0010 
uint32      COMMONGROUP_ANGULARRATE		= 0x0020 
uint32      COMMONGROUP_POSITION		= 0x0040 
uint32      COMMONGROUP_VELOCITY		= 0x0080 
uint32      COMMONGROUP_ACCEL			= 0x0100 
uint32      COMMONGROUP_IMU				= 0x0200 
uint32      COMMONGROUP_MAGPRES			= 0x0400 
uint32      COMMONGROUP_DELTATHETA		= 0x0800 
uint32      COMMONGROUP_INSSTATUS		= 0x1000 
uint32      COMMONGROUP_SYNCINCNT		= 0x2000 
uint32      COMMONGROUP_TIMEGPSPPS		= 0x4000 

uint32      TIMEGROUP_NONE				= 0x0000
uint32      TIMEGROUP_TIMESTARTUP		= 0x0001
uint32      TIMEGROUP_TIMEGPS			= 0x0002
uint32      TIMEGROUP_GPSTOW			= 0x0004
uint32      TIMEGROUP_GPSWEEK			= 0x0008
uint32      TIMEGROUP_TIMESYNCIN		= 0x0010
uint32      TIMEGROUP_TIMEGPSPPS		= 0x0020
uint32      TIMEGROUP_TIMEUTC			= 0x0040
uint32      TIMEGROUP_SYNCINCNT			= 0x0080
uint32      TIMEGROUP_SYNCOUTCNT        = 0x0100
uint32      TIMEGROUP_TIMESTATUS        = 0x0200

uint32      IMUGROUP_NONE				= 0x0000
uint32      IMUGROUP_IMUSTATUS			= 0x0001
uint32      IMUGROUP_UNCOMPMAG			= 0x0002
uint32      IMUGROUP_UNCOMPACCEL		= 0x0004
uint32      IMUGROUP_UNCOMPGYRO			= 0x0008
uint32      IMUGROUP_TEMP				= 0x0010
uint32      IMUGROUP_PRES				= 0x0020
uint32      IMUGROUP_DELTATHETA			= 0x0040
uint32      IMUGROUP_DELTAVEL			= 0x0080
uint32      IMUGROUP_MAG				= 0x0100
uint32      IMUGROUP_ACCEL				= 0x0200
uint32      IMUGROUP_ANGULARRATE		= 0x0400
uint32      IMUGROUP_SENSSAT			= 0x0800

uint32      GPSGROUP_NONE			    = 0x0000
uint32      GPSGROUP_UTC			    = 0x0001
uint32      GPSGROUP_TOW			    = 0x0002
uint32      GPSGROUP_WEEK			    = 0x0004
uint32      GPSGROUP_NUMSATS		    = 0x0008
uint32      GPSGROUP_FIX			    = 0x0010
uint32      GPSGROUP_POSLLA			    = 0x0020
uint32      GPSGROUP_POSECEF		    = 0x0040
uint32      GPSGROUP_VELNED			    = 0x0080
uint32      GPSGROUP_VELECEF		    = 0x0100
uint32      GPSGROUP_POSU			    = 0x0200
uint32      GPSGROUP_VELU			    = 0x0400
uint32      GPSGROUP_TIMEU			    = 0x0800
uint32      GPSGROUP_TIMEINFO           = 0x1000
uint32      GPSGROUP_DOP                = 0x2000

uint32      ATTITUDEGROUP_NONE				= 0x0000
uint32      ATTITUDEGROUP_VPESTATUS			= 0x0001
uint32      ATTITUDEGROUP_YAWPITCHROLL		= 0x0002
uint32      ATTITUDEGROUP_QUATERNION		= 0x0004 
uint32      ATTITUDEGROUP_DCM				= 0x0008
uint32      ATTITUDEGROUP_MAGNED			= 0x0010
uint32      ATTITUDEGROUP_ACCELNED			= 0x0020
uint32      ATTITUDEGROUP_LINEARACCELBODY	= 0x0040
uint32      ATTITUDEGROUP_LINEARACCELNED	= 0x0080
uint32      ATTITUDEGROUP_YPRU				= 0x0100

uint32      INSGROUP_NONE					= 0x0000
uint32      INSGROUP_INSSTATUS				= 0x0001
uint32      INSGROUP_POSLLA					= 0x0002
uint32      INSGROUP_POSECEF				= 0x0004
uint32      INSGROUP_VELBODY				= 0x0008
uint32      INSGROUP_VELNED					= 0x0010
uint32      INSGROUP_VELECEF				= 0x0020
uint32      INSGROUP_MAGECEF				= 0x0040
uint32      INSGROUP_ACCELECEF				= 0x0080
uint32      INSGROUP_LINEARACCELECEF		= 0x0100
uint32      INSGROUP_POSU					= 0x0200
uint32      INSGROUP_VELU					= 0x0400





