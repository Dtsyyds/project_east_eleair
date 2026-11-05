# Image acquisition time; frame_id is the camera optical frame.
std_msgs/Header header

# Monotonically increasing within one run.
uint64 frame_seq

# Both errors are reliable; ignore both errors when false.
bool valid

# Dimensionless image lateral error: left negative, right positive.
float64 lateral_error_norm

# Image heading in radians: forward-left positive, forward-right negative.
float64 heading_error_rad

# Common recognition confidence for both errors, in [0, 1].
float32 confidence
