std_msgs/Header header

# * Hardware command
float64[] joint_pos_target   # * only for servo test
float64[] joint_vel_target   # * only for servo test
float64[] joint_tau_target   # * only for servo test

uint16[] controlword

bool ctrl_on
bool traj_on
bool motor_offset

float64[] kp_x
float64[] ki_x
float64[] kd_x
float64[] d_cutoff_x

float64[] kp_y
float64[] ki_y
float64[] kd_y
float64[] d_cutoff_y

float64[] kp_z
float64[] ki_z
float64[] kd_z
float64[] d_cutoff_z


# * GUI widget command
int32 test_mode  # 0 (IDLE), 1 (AXIS), 2 (CONTROL)

bool b_all_joints_enabled
bool[] b_joint_enabled
bool[] b_joints_target_cmd_on
bool b_control_input_started
bool b_reference_started
bool b_estop_button_pressed

# * Control parameters
float64[] joint_kp
float64[] joint_kd

float64 ref_sine_amp
float64 ref_sine_freq
int32 ref_sine_repeat
# float64 ref_sine_interval
