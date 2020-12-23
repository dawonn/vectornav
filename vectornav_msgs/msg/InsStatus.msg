# VectorNav Time Status
# UM005 - 10.2.2

# TODO[Dereck] VNCXX does not match manual exactly

uint8 mode           
bool  gps_fix        			
bool  time_error     		
bool  imu_error      	
bool  mag_pres_error 
bool  gps_error      
bool  gps_heading_ins
bool  gps_compass    

uint8 MODE_NOT_TRACKING     = 0		
uint8 MODE_ALIGNING         = 1
uint8 MODE_TRACKING         = 2
uint8 MODE_NO_GPS           = 3	
