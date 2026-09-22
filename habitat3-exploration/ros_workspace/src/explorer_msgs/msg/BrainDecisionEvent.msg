std_msgs/Header header
string brain_id
# navigate | wait | complete
string action
uint32 goal_id
float32 goal_x
float32 goal_y
string detail
uint32[] visited_ids
uint32[] live_ids
# Robot pose at decision time (map frame) + Euclidean distance to goal.
float32 robot_x
float32 robot_y
float32 goal_distance_m
