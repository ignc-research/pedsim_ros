# Added by Ronja Gueldenring
# For spawning agents dynamically in pedsim and forwarding them to flatland
int16 id
geometry_msgs/Point pos
string type
int16 number_of_peds
float64 vmax

float64 chatting_probability
float64 tell_story_probability
float64 group_talking_probability
float64 talking_and_walking_probability
float64 requesting_service_probability

float64 max_talking_distance
float64 max_servicing_radius

float64 talking_base_time
float64 tell_story_base_time
float64 group_talking_base_time
float64 talking_and_walking_base_time
float64 state_receiving_service_base_time
float64 state_requesting_service_base_time

# forces
float64 force_factor_desired
float64 force_factor_obstacle
float64 force_factor_social

geometry_msgs/Point[] waypoints
int16 waypoint_mode

string yaml_file

