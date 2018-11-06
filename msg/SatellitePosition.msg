Header header
inertial_sense/GTime time
geometry_msgs/Vector3 position # sat position ECEF (m)
geometry_msgs/Vector3 velocity # sat velocity ECEF (m/s)
float64 clock_bias # (s)
float64 clock_drift # (s/s)
float64 var # sat position and clock error variance (m^2)
int64 health # sat health flag