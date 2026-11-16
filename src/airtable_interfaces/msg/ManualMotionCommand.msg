# Motion directions are in base_link: +X forward, +Y left, +yaw CCW.
# RW_ROTATE_* refers to wheel rotation; the body receives the opposite torque.
uint8 STOP=0
uint8 FORWARD=1
uint8 BACKWARD=2
uint8 LEFT=3
uint8 RIGHT=4
uint8 THRUSTER_ROTATE_CCW=5
uint8 THRUSTER_ROTATE_CW=6
uint8 RW_ROTATE_CCW=7
uint8 RW_ROTATE_CW=8
# Toggle yaw attitude hold by the reaction wheel (rw_attitude node).
uint8 RW_ATTITUDE_TOGGLE=9

builtin_interfaces/Time stamp
uint8 command
builtin_interfaces/Duration pulse_duration
