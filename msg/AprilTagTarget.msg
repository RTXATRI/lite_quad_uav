int32 tag_id
string state

bool detected
bool pose_valid

# body_frame: camera -> body extrinsic transformed raw tag pose.
geometry_msgs/Pose pose_body_raw

# level_frame: raw body position with roll/pitch compensation applied.
# If IMU is stale, this degrades to pose_body_raw.position.
geometry_msgs/Point position_level_raw

# level_frame: alpha-beta filtered position_level_raw.
geometry_msgs/Point position_level_filtered

float64 confidence
float64 detection_age
float64 imu_age
float64 residual_norm

uint32 stable_count
uint32 lost_count
