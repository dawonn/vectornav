std_msgs/Header       header
float64               Time
uint16                Week
uint16                Status
geometry_msgs/Vector3 RPY
geometry_msgs/Vector3 LLA
geometry_msgs/Vector3 NedVel
float32               YawUncertainty
float32               PitchUncertainty
float32               RollUncertainty
float32               PosUncertainty
float32               VelUncertainty
float64               SyncInTime
uint32                SyncInCount

uint16 STATUS_INSUFFICIENT_MOTION=1
uint16 STATUS_INS_OK=2
uint16 STATUS_GPS_FIX_OK=4
uint16 STATUS_SENSOR_ERROR_TIME=8
uint16 STATUS_SENSOR_ERROR_IMU=16
uint16 STATUS_SENSOR_ERROR_MAG=32
uint16 STATUS_SENSOR_ERROR_GPS=64


