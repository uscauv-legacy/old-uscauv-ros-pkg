Header header

string type

# Current color of the object. Fixed for static objects like path segments, but not for buoys
string color

geometry_msgs/PoseWithCovariance pose

float64 variance

float64 symmetry

int64[] child_idx

# Whether or not this object has more confidence than all other objects of the same type
bool is_best_estimate