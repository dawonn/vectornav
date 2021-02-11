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

uint16                      IMUGROUP_IMUSTATUS              = 0x0001
uint16                      IMUGROUP_UNCOMPMAG              = 0x0002
uint16                      IMUGROUP_UNCOMPACCEL            = 0x0004
uint16                      IMUGROUP_UNCOMPGYRO             = 0x0008
uint16                      IMUGROUP_TEMP                   = 0x0010
uint16                      IMUGROUP_PRES                   = 0x0020
uint16                      IMUGROUP_DELTATHETA             = 0x0040
uint16                      IMUGROUP_DELTAVEL               = 0x0080
uint16                      IMUGROUP_MAG                    = 0x0100
uint16                      IMUGROUP_ACCEL                  = 0x0200
uint16                      IMUGROUP_ANGULARRATE            = 0x0400
uint16                      IMUGROUP_SENSSAT                = 0x0800
# TODO[Dereck] SENSSAT exists in the header, but not the manual...


# Fields
uint16                      imustatus
geometry_msgs/Vector3       uncompmag
geometry_msgs/Vector3       uncompaccel
geometry_msgs/Vector3       uncompgyro
float32                     temp
float32                     pres
float32                     deltatheta_time
geometry_msgs/Vector3       deltatheta_dtheta
geometry_msgs/Vector3       deltavel
geometry_msgs/Vector3       mag
geometry_msgs/Vector3       accel
geometry_msgs/Vector3       angularrate
uint16                      sensat
