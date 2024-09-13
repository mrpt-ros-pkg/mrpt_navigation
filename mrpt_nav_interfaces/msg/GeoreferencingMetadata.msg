# For a diagram of the frames defined in this message, refer to online docs: 
# https://docs.mola-slam.org/latest/geo-referencing.html

# This is true only if the map is georeferenced. If it is false,
# the rest of the fields in this message should be ignored.
bool valid

geometry_msgs/PoseWithCovariance t_enu_to_map
geometry_msgs/Pose t_enu_to_utm

# The geodetic coordinates of the ENU reference frame:
float64 latitude
float64 longitude
float64 height

# UTM zone, for the ENU point.
int32 utm_zone
string utm_band  # just one letter (or empty if valid=false)
