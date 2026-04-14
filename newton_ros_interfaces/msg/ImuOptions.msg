# Configuration for an IMU sensor
GeneralSensorOptions options
RosSensorOptions ros_options
RigidSensorOptions rigid_options
NoisySensorOptions noisy_options

# Accelerometer parameters
float32 acc_resolution
float32[] acc_cross_axis_coupling
float32[] acc_bias
float32[] acc_noise
float32[] acc_random_walk

# Gyroscope parameters
float32 gyro_resolution
float32[] gyro_cross_axis_coupling
float32[] gyro_bias
float32[] gyro_noise
float32[] gyro_random_walk

# Debug visualization
float32[] debug_acc_color
float32 debug_acc_scale
float32[] debug_gyro_color
float32 debug_gyro_scale
