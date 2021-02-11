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

uint16                      COMMONGROUP_TIMESTARTUP         = 0x0001
uint16                      COMMONGROUP_TIMEGPS             = 0x0002
uint16                      COMMONGROUP_TIMESYNCIN          = 0x0004
uint16                      COMMONGROUP_YAWPITCHROLL        = 0x0008
uint16                      COMMONGROUP_QUATERNION          = 0x0010
uint16                      COMMONGROUP_ANGULARRATE         = 0x0020
uint16                      COMMONGROUP_POSITION            = 0x0040
uint16                      COMMONGROUP_VELOCITY            = 0x0080
uint16                      COMMONGROUP_ACCEL               = 0x0100
uint16                      COMMONGROUP_IMU                 = 0x0200
uint16                      COMMONGROUP_MAGPRES             = 0x0400
uint16                      COMMONGROUP_DELTATHETA          = 0x0800
uint16                      COMMONGROUP_INSSTATUS           = 0x1000
uint16                      COMMONGROUP_SYNCINCNT           = 0x2000
uint16                      COMMONGROUP_TIMEGPSPPS          = 0x4000

# Common
uint64                      timestartup
uint64                      timegps
uint64                      timesyncin
geometry_msgs/Vector3       yawpitchroll
geometry_msgs/Quaternion    quaternion
geometry_msgs/Vector3       angularrate
geometry_msgs/Point         position
geometry_msgs/Vector3       velocity
geometry_msgs/Vector3       accel
geometry_msgs/Vector3       imu_accel
geometry_msgs/Vector3       imu_rate
geometry_msgs/Vector3       magpres_mag
float32                     magpres_temp
float32                     magpres_pres
float32                     deltatheta_dtime
geometry_msgs/Vector3       deltatheta_dtheta
geometry_msgs/Vector3       deltatheta_dvel
vectornav_msgs/InsStatus    insstatus
uint32                      syncincnt
uint16                      timegpspps
