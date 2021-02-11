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

uint16                      GPSGROUP_UTC                    = 0x0001
uint16                      GPSGROUP_TOW                    = 0x0002
uint16                      GPSGROUP_WEEK                   = 0x0004
uint16                      GPSGROUP_NUMSATS                = 0x0008
uint16                      GPSGROUP_FIX                    = 0x0010
uint16                      GPSGROUP_POSLLA                 = 0x0020
uint16                      GPSGROUP_POSECEF                = 0x0040
uint16                      GPSGROUP_VELNED                 = 0x0080
uint16                      GPSGROUP_VELECEF                = 0x0100
uint16                      GPSGROUP_POSU                   = 0x0200
uint16                      GPSGROUP_VELU                   = 0x0400
uint16                      GPSGROUP_TIMEU                  = 0x0800
uint16                      GPSGROUP_TIMEINFO               = 0x1000
uint16                      GPSGROUP_DOP                    = 0x2000
# TODO[Dereck] vncxx does not support SatInfo or RawMeas

# Fields
vectornav_msgs/TimeUTC      utc
uint64                      tow
uint16                      week
uint8                       numsats
uint8                       fix
geometry_msgs/Point         poslla
geometry_msgs/Point         posecef
geometry_msgs/Vector3       velned
geometry_msgs/Vector3       velecef
geometry_msgs/Vector3       posu
float32                     velu
uint32                      timeu
uint8                       timeinfo_status
int8                        timeinfo_leapseconds
vectornav_msgs/DOP          dop

uint8                       GPSFIX_NOFIX                    = 0
uint8                       GPSFIX_TIMEONLY                 = 1
uint8                       GPSFIX_2D                       = 2
uint8                       GPSFIX_3D                       = 3
