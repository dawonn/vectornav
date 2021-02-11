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

uint16                      TIMEGROUP_TIMESTARTUP           = 0x0001
uint16                      TIMEGROUP_TIMEGPS               = 0x0002
uint16                      TIMEGROUP_GPSTOW                = 0x0004
uint16                      TIMEGROUP_GPSWEEK               = 0x0008
uint16                      TIMEGROUP_TIMESYNCIN            = 0x0010
uint16                      TIMEGROUP_TIMEGPSPPS            = 0x0020
uint16                      TIMEGROUP_TIMEUTC               = 0x0040
uint16                      TIMEGROUP_SYNCINCNT             = 0x0080
uint16                      TIMEGROUP_SYNCOUTCNT            = 0x0100
uint16                      TIMEGROUP_TIMESTATUS            = 0x0200

# Fields
uint64                      timestartup
uint64                      timegps
uint64                      gpstow
uint16                      gpsweek
uint64                      timesyncin
uint64                      timegpspps
vectornav_msgs/TimeUTC      timeutc
uint32                      syncincnt
uint32                      syncoutcnt
vectornav_msgs/TimeStatus   timestatus
