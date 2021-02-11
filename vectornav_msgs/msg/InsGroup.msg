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

uint16                      INSGROUP_INSSTATUS              = 0x0001
uint16                      INSGROUP_POSLLA                 = 0x0002
uint16                      INSGROUP_POSECEF                = 0x0004
uint16                      INSGROUP_VELBODY                = 0x0008
uint16                      INSGROUP_VELNED                 = 0x0010
uint16                      INSGROUP_VELECEF                = 0x0020
uint16                      INSGROUP_MAGECEF                = 0x0040
uint16                      INSGROUP_ACCELECEF              = 0x0080
uint16                      INSGROUP_LINEARACCELECEF        = 0x0100
uint16                      INSGROUP_POSU                   = 0x0200
uint16                      INSGROUP_VELU                   = 0x0400


# Fields
vectornav_msgs/InsStatus    insstatus
geometry_msgs/Point         poslla
geometry_msgs/Point         posecef
geometry_msgs/Vector3       velbody
geometry_msgs/Vector3       velned
geometry_msgs/Vector3       velecef
geometry_msgs/Vector3       magecef
geometry_msgs/Vector3       accelecef
geometry_msgs/Vector3       linearaccelecef
float32                     posu
float32                     velu