# Normalized wheel speed command for speed-type wheels without torque control,
# such as a continuous-rotation servo standing in for an ARW-3m.
# speeds are in [-1, 1] of the wheel's calibrated full speed; 0 stops the wheel.
# Positive is wheel rotation CCW about base_link +z (same sense as RW_ROTATE_CCW);
# the body receives the opposite reaction.
builtin_interfaces/Time stamp
string[] names
float64[] speeds
