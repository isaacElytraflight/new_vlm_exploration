# Structured NEW-frontier navigation failure diagnostics (log-only; no control effect).
std_msgs/Header header

# classified | resolved
string stage
# inaccessible | stuck (at classified); mark_dead | terminate_stuck | recovered | retry_failed_mark_dead (at resolved)
string fail_class
string resolution

uint32 goal_id
float32 goal_x
float32 goal_y
uint32 prior_id

float32 robot_x
float32 robot_y

float32 nav_dt_s
uint16 nav_error_code
string nav_error

bool prior_pose_known
bool prior_plan_ok
bool new_goal_plan_ok
uint16 new_plan_code
string new_plan_err
bool definitive_unreachable

float32 start_clearance_m
bool start_clearance_ok

# Occupancy from /grid_map (-128 = off-map / unavailable)
int8 grid_occ_at_robot
int8 grid_occ_at_goal

# Nav2 global costmap cell (0-254 typical; -1 = unavailable)
int16 costmap_at_robot
int16 costmap_at_goal
