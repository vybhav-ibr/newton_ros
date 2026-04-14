# Configuration for a contact force sensor
GeneralSensorOptions options
RosSensorOptions ros_options
RigidSensorOptions rigid_options
NoisySensorOptions noisy_options

float32[] min_force     # Lower bounds for force measurement
float32[] max_force     # Upper bounds for force measurement
float32[] debug_color   # visualization color [r, g, b, a]
float32 debug_scale     # Scale of debug vectors
