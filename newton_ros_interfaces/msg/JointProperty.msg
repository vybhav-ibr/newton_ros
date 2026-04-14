# Physics and control properties for a robot joint
string name
float32 kp            # Proportional gain
float32 kv            # Velocity gain
float32 stiffness
float32 armature
float32 damping
float32[] force_range  # [min_force, max_force]
string command         # Control mode: position, velocity, or effort
