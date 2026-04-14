# Configuration options for a camera sensor
GeneralSensorOptions options
RosSensorOptions ros_options
RigidSensorOptions rigid_options

string[] camera_types    # e.g., ["rgb", "depth"]
int32[] res             # [width, height]
float32 fov              # Field of view in degrees

float32 near             # Near clipping plane
float32 far              # Far clipping plane
float32 aperture
bool denoise
int32 spp                # Samples per pixel
string model             # e.g., "pinhole"
float32 focus_dist
bool gui                 # Visibility in simulation GUI
bool add_noise
float32 noise_mean
float32 noise_std
