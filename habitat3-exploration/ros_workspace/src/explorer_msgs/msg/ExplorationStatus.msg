std_msgs/Header header
string phase
uint32 current_node_id
uint32 target_node_id
bool exploration_complete
string detail
# How the episode ended when exploration_complete is true (success|stuck|timeout|...).
string termination_reason
