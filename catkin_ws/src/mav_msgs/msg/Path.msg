Header header
bool line # determines whether the path is a line 
# true is line
# false is circle
float32 Vg # Desired groundspeed
geometry_msgs/Vector3 r # origin of linear path
geometry_msgs/Vector3 q # direction of linear path
geometry_msgs/Vector3 c # orbit center
float32 rho # radius of the orbit
float32 lamb # direction for orbit (1 for clockwise, -1 for counterclockwise)
