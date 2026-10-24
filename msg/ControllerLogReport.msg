std_msgs/Header header

string controller_name
string node_state
string state
uint8 controller_type
uint8 mix_mode
string source_task
string source_action

bool command_received
bool command_valid
bool command_fresh
bool vehicle_state_valid
bool vehicle_status_fresh
bool local_position_fresh
bool reliable_vehicle_connection
bool target_state_valid
bool rl_tracking_valid
bool rl_landing_valid

bool output_valid
float64 output_vx
float64 output_vy
float64 output_vz
float64 output_yaw_rate

float64 position_error_norm
float64 target_distance
float64 rl_weight
bool converged
bool saturated
bool fallback_active

string message
