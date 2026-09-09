std_msgs/Header header
string brain_id
# Parallel arrays: edge from_ids[i] -> to_ids[i] with Nav2 (or proxy) costs[i].
uint32[] from_ids
uint32[] to_ids
float32[] costs
