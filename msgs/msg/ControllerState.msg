std_msgs/Header header

sensor_msgs/JointState joint_state
sensor_msgs/JointState joint_state_des


float64[] leg_pos
float64[] des_leg_pos

float64[] leg_vel
float64[] des_leg_vel


uint16[] statusword
int8[] mode_of_operation_disp
# uint32[] digital_inputs
# uint32[] dclink_circuit_voltage
# int16[] analog_input


uint16[] slave_state
uint16 expected_wkc
uint16 wkc

float64 test_sampling_period_ms

bool b_ecat_thread_start
bool b_elmo_thread_start
bool b_ctrl_thread_start
bool b_ecat_loop_running
bool b_elmo_loop_running
bool b_ctrl_loop_running
bool b_system_ready

float64 ecat_sampling_period_ms
float64 ecat_jitter_ms
uint32 ecat_overrun_cnt
float64 ecat_loop_time_sec

float64 elmo_sampling_period_ms
float64 elmo_jitter_ms
uint32 elmo_overrun_cnt
float64 elmo_loop_time_sec

float64 ctrl_sampling_period_ms
float64 ctrl_jitter_ms
uint32 ctrl_overrun_cnt
float64 ctrl_loop_time_sec

