# VectorNav Composite Data Packet
#
# Only the enabled fields will be populated with data and only the fields 
# you require for your application should be enabled to conserve bandwidth
# and increase the update rate.
#
# TODO[Dereck] Requires rosidl patches

# Data Received (ROS Time)
std_msgs/Header header       

# Field Enable Bits
uint16                      group_fields

uint16                      ATTITUDEGROUP_VPESTATUS         = 0x0001
uint16                      ATTITUDEGROUP_YAWPITCHROLL      = 0x0002
uint16                      ATTITUDEGROUP_QUATERNION        = 0x0004 
uint16                      ATTITUDEGROUP_DCM               = 0x0008
uint16                      ATTITUDEGROUP_MAGNED            = 0x0010
uint16                      ATTITUDEGROUP_ACCELNED          = 0x0020
uint16                      ATTITUDEGROUP_LINEARACCELBODY   = 0x0040
uint16                      ATTITUDEGROUP_LINEARACCELNED    = 0x0080
uint16                      ATTITUDEGROUP_YPRU              = 0x0100

# Fields 
vectornav_msgs/VpeStatus    vpestatus         
geometry_msgs/Vector3       yawpitchroll      
geometry_msgs/Quaternion    quaternion       
float32[9]                  dcm            
geometry_msgs/Vector3       magned         
geometry_msgs/Vector3       accelned         
geometry_msgs/Vector3       linearaccelbody   
geometry_msgs/Vector3       linearaccelned   
geometry_msgs/Vector3       ypru            
