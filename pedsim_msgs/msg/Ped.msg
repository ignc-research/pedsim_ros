# Added by Ronja Gueldenring
# For spawning agents dynamically in pedsim and forwarding them to flatland
int16 id
geometry_msgs/Point pos
int16 type
int16 number_of_peds
float64 vmax
float64 chatting_probability
int16 waypoint_mode

# forces
float64 force_factor_desired
float64 force_factor_obstacle
float64 force_factor_social

# regarding waypoints:
# x, y: position
# z: radius of waypoint (determines how close a ped needs to be to a waypoint in order to reach it)
geometry_msgs/Point[] waypoints

string yaml_file

