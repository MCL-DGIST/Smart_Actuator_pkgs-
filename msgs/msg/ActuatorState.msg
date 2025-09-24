# mcl_quadruped_msgs/msg/ActuatorState.msg
builtin_interfaces/Time stamp
float64[] pos_motor
float64[] vel_motor

float64[] joint_pos_target
float64[] pos_load
float64[] vel_load


float64[] joint_angle
float64[] joint_vel

float64[] current



bool system_ready

