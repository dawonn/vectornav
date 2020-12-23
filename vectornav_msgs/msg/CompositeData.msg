# VectorNav Composite Data Packet
#
# Only the enabled fields will be populated with data and only the fields 
# you require for your application should be enabled to conserve bandwidth
# and increase the update rate.
#
# TODO[Dereck] Requires rosidl patches

# Data Received (ROS Time)
std_msgs/Header header

# Group Enable Bits
uint16	                    groups

uint16	                    BINARYGROUP_COMMON              = 0x01		
uint16	                    BINARYGROUP_TIME                = 0x02	
uint16	                    BINARYGROUP_IMU                 = 0x04	
uint16	                    BINARYGROUP_GPS                 = 0x08	
uint16	                    BINARYGROUP_ATTITUDE            = 0x10	
uint16	                    BINARYGROUP_INS                 = 0x20		
uint16                      BINARYGROUP_GPS2                = 0x40			

# Field Enable Bits
uint16[<=8]                 group_fields

uint16                      COMMONGROUP_TIMESTARTUP		    = 0x0001 
uint16                      COMMONGROUP_TIMEGPS			    = 0x0002 
uint16                      COMMONGROUP_TIMESYNCIN		    = 0x0004 
uint16                      COMMONGROUP_YAWPITCHROLL	    = 0x0008 
uint16                      COMMONGROUP_QUATERNION		    = 0x0010 
uint16                      COMMONGROUP_ANGULARRATE		    = 0x0020 
uint16                      COMMONGROUP_POSITION		    = 0x0040 
uint16                      COMMONGROUP_VELOCITY		    = 0x0080 
uint16                      COMMONGROUP_ACCEL			    = 0x0100 
uint16                      COMMONGROUP_IMU				    = 0x0200 
uint16                      COMMONGROUP_MAGPRES			    = 0x0400 
uint16                      COMMONGROUP_DELTATHETA		    = 0x0800 
uint16                      COMMONGROUP_INSSTATUS		    = 0x1000 
uint16                      COMMONGROUP_SYNCINCNT		    = 0x2000 
uint16                      COMMONGROUP_TIMEGPSPPS		    = 0x4000 

uint16                      TIMEGROUP_TIMESTARTUP		    = 0x0001
uint16                      TIMEGROUP_TIMEGPS			    = 0x0002
uint16                      TIMEGROUP_GPSTOW			    = 0x0004
uint16                      TIMEGROUP_GPSWEEK			    = 0x0008
uint16                      TIMEGROUP_TIMESYNCIN		    = 0x0010
uint16                      TIMEGROUP_TIMEGPSPPS		    = 0x0020
uint16                      TIMEGROUP_TIMEUTC			    = 0x0040
uint16                      TIMEGROUP_SYNCINCNT			    = 0x0080
uint16                      TIMEGROUP_SYNCOUTCNT            = 0x0100
uint16                      TIMEGROUP_TIMESTATUS            = 0x0200

uint16                      IMUGROUP_IMUSTATUS			    = 0x0001
uint16                      IMUGROUP_UNCOMPMAG			    = 0x0002
uint16                      IMUGROUP_UNCOMPACCEL		    = 0x0004
uint16                      IMUGROUP_UNCOMPGYRO			    = 0x0008
uint16                      IMUGROUP_TEMP				    = 0x0010
uint16                      IMUGROUP_PRES				    = 0x0020
uint16                      IMUGROUP_DELTATHETA			    = 0x0040
uint16                      IMUGROUP_DELTAVEL			    = 0x0080
uint16                      IMUGROUP_MAG				    = 0x0100
uint16                      IMUGROUP_ACCEL				    = 0x0200
uint16                      IMUGROUP_ANGULARRATE		    = 0x0400
uint16                      IMUGROUP_SENSSAT		        = 0x0800
# TODO[Dereck] SENSSAT exists in the header, but not the manual...

uint16                      GPSGROUP_UTC			        = 0x0001
uint16                      GPSGROUP_TOW			        = 0x0002
uint16                      GPSGROUP_WEEK			        = 0x0004
uint16                      GPSGROUP_NUMSATS		        = 0x0008
uint16                      GPSGROUP_FIX			        = 0x0010
uint16                      GPSGROUP_POSLLA			        = 0x0020
uint16                      GPSGROUP_POSECEF		        = 0x0040
uint16                      GPSGROUP_VELNED			        = 0x0080
uint16                      GPSGROUP_VELECEF		        = 0x0100
uint16                      GPSGROUP_POSU			        = 0x0200
uint16                      GPSGROUP_VELU			        = 0x0400
uint16                      GPSGROUP_TIMEU			        = 0x0800
uint16                      GPSGROUP_TIMEINFO               = 0x1000
uint16                      GPSGROUP_DOP                    = 0x2000
# TODO[Dereck] vncxx does not support SatInfo or RawMeas

uint16                      ATTITUDEGROUP_VPESTATUS			= 0x0001
uint16                      ATTITUDEGROUP_YAWPITCHROLL		= 0x0002
uint16                      ATTITUDEGROUP_QUATERNION		= 0x0004 
uint16                      ATTITUDEGROUP_DCM				= 0x0008
uint16                      ATTITUDEGROUP_MAGNED			= 0x0010
uint16                      ATTITUDEGROUP_ACCELNED			= 0x0020
uint16                      ATTITUDEGROUP_LINEARACCELBODY	= 0x0040
uint16                      ATTITUDEGROUP_LINEARACCELNED	= 0x0080
uint16                      ATTITUDEGROUP_YPRU				= 0x0100

uint16                      INSGROUP_INSSTATUS				= 0x0001
uint16                      INSGROUP_POSLLA					= 0x0002
uint16                      INSGROUP_POSECEF				= 0x0004
uint16                      INSGROUP_VELBODY				= 0x0008
uint16                      INSGROUP_VELNED					= 0x0010
uint16                      INSGROUP_VELECEF				= 0x0020
uint16                      INSGROUP_MAGECEF				= 0x0040
uint16                      INSGROUP_ACCELECEF				= 0x0080
uint16                      INSGROUP_LINEARACCELECEF		= 0x0100
uint16                      INSGROUP_POSU					= 0x0200
uint16                      INSGROUP_VELU					= 0x0400

#
# Fields 
#

# Common
uint64                      common_timestartup	
uint64                      common_timegps		 
uint64                      common_timesyncin	 
geometry_msgs/Vector3       common_yawpitchroll 
geometry_msgs/Quaternion    common_quaternion	 
geometry_msgs/Vector3       common_angularrate	 
geometry_msgs/Point         common_position	 
geometry_msgs/Vector3       common_velocity	 
geometry_msgs/Vector3       common_accel
geometry_msgs/Vector3       common_imu_accel	 
geometry_msgs/Vector3       common_imu_rate			 
geometry_msgs/Vector3       common_magpres_mag			 
float32                     common_magpres_temp			 
float32                     common_magpres_pres		 
uint16                      common_deltatheta_dtime			 
geometry_msgs/Vector3       common_deltatheta_dtheta			 
geometry_msgs/Vector3       common_deltatheta_dvel	 
vectornav_msgs/InsStatus    common_insstatus	 
uint32                      common_syncincnt	 
uint16                      common_timegpspps	 

# TIME
uint64                      time_timestartup		
uint64                      time_timegps			
uint64                      time_gpstow			
uint16                      time_gpsweek			
uint64                      time_timesyncin		
uint64                      time_timegpspps		
vectornav_msgs/TimeUTC      time_timeutc			
uint32                      time_syncincnt			
uint32                      time_syncoutcnt        
vectornav_msgs/TimeStatus   time_timestatus

# IMU
uint16                      imu_imustatus			
geometry_msgs/Vector3       imu_uncompmag			
geometry_msgs/Vector3       imu_uncompaccel		
geometry_msgs/Vector3       imu_uncompgyro			
float32                     imu_temp				
float32                     imu_pres
float32                     imu_deltatheta_time	
geometry_msgs/Vector3       imu_deltatheta_dtheta			
geometry_msgs/Vector3       imu_deltavel			
geometry_msgs/Vector3       imu_mag				
geometry_msgs/Vector3       imu_accel				
geometry_msgs/Vector3       imu_angularrate			
uint16                      imu_sensat

# GPS
vectornav_msgs/TimeUTC      gps_utc			    
uint64                      gps_tow			    
uint16                      gps_week			    
uint8                       gps_numsats		    
uint8                       gps_fix			    
geometry_msgs/Point         gps_poslla			    
geometry_msgs/Point         gps_posecef		    
geometry_msgs/Vector3       gps_velned			    
geometry_msgs/Vector3       gps_velecef		    
geometry_msgs/Vector3       gps_posu			    
float32                     gps_velu			    
uint32                      gps_timeu
uint8                       gps_timeinfo_status
int8                        gps_timeinfo_leapseconds
vectornav_msgs/DOP          gps_dop

uint8                       GPSFIX_NOFIX                    = 0
uint8                       GPSFIX_TIMEONLY                 = 1
uint8                       GPSFIX_2D                       = 2
uint8                       GPSFIX_3D                       = 3

# Attitude
vectornav_msgs/VpeStatus    attitude_vpestatus			
geometry_msgs/Vector3       attitude_yawpitchroll		
geometry_msgs/Quaternion    attitude_quaternion		 
float32[9]                  attitude_dcm				
geometry_msgs/Vector3       attitude_magned			
geometry_msgs/Vector3       attitude_accelned			
geometry_msgs/Vector3       attitude_linearaccelbody	
geometry_msgs/Vector3       attitude_linearaccelned	
geometry_msgs/Vector3       attitude_ypru				

# INS
vectornav_msgs/InsStatus    ins_insstatus		
geometry_msgs/Point         ins_poslla			
geometry_msgs/Point         ins_posecef		
geometry_msgs/Vector3       ins_velbody		
geometry_msgs/Vector3       ins_velned			
geometry_msgs/Vector3       ins_velecef		
geometry_msgs/Vector3       ins_magecef		
geometry_msgs/Vector3       ins_accelecef		
geometry_msgs/Vector3       ins_linearaccelecef
float32                     ins_posu			
float32                     ins_velu			

# GPS2
vectornav_msgs/TimeUTC      gps2_utc			    
uint64                      gps2_tow			    
uint16                      gps2_week			    
uint8                       gps2_numsats		    
uint8                       gps2_fix			    
geometry_msgs/Point         gps2_poslla			    
geometry_msgs/Point         gps2_posecef		    
geometry_msgs/Vector3       gps2_velned			    
geometry_msgs/Vector3       gps2_velecef		    
geometry_msgs/Vector3       gps2_posu			    
float32                     gps2_velu			    
uint32                      gps2_timeu
uint8                       gps2_timeinfo_status
int8                        gps2_timeinfo_leapseconds
vectornav_msgs/DOP          gps2_dop