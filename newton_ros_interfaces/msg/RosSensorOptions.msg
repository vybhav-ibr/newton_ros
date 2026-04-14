# ROS-specific sensor configuration
string frame_id          # TF frame name
float32 frequency        # Publishing frequency
string topic             # ROS topic name
string qos_history       # keep_last or keep_all
int32 qos_depth
string qos_reliability   # reliable or best_effort
string qos_durability    # volatile or transient_local
