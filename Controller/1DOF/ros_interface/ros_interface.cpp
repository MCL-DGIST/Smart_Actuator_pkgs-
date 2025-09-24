#include "ros_interface.hpp"
#include "ecat_func.hpp"

#include <iostream>

using namespace std;

ElmoRosInterface::ElmoRosInterface() : Node("elmo_ros_interface") {
  RCLCPP_INFO(this->get_logger(), "Starting ELMO ROS Interface initialization...");

  //* initialize shared memory manager (accessor mode) *//
  shm_manager_ = nullptr;
  try { // try to connect to shared memory
    RCLCPP_INFO(this->get_logger(), "Connecting to shared memory '%s'...", SHM_NAME);

    shm_manager_ = &InterProcessStateManager::getInstance(false); // open shared memory
    RCLCPP_INFO(this->get_logger(), "Connected to shared memory successfully");

    /* check system ready status */
    { // ! scope is important for automatic lock/unlock
      auto shm = shm_manager_->getSafeAccess();
      b_system_ready_ = shm->b_system_ready;
      RCLCPP_INFO(this->get_logger(), "Is system ready?: %s", b_system_ready_ ? "TRUE" : "FALSE");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to shared memory: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Exception type: %s", typeid(e).name());
    RCLCPP_ERROR(this->get_logger(), "Possible causes:");
    RCLCPP_ERROR(this->get_logger(), "1. Real-time controller not running");
    RCLCPP_ERROR(this->get_logger(), "2. Permission issue: check if user can access shared memory");
    RCLCPP_ERROR(this->get_logger(), "3. Shared memory not created properly");

    system("echo 'Shared memory files:' && ls -la /dev/shm/ | grep elmo"); // Check system status

    shm_manager_ = nullptr;
  }

    

  //! INIT massage variable !//
    // msg_actuator_.pos_load.resize(NUMOFSLAVES);
    // msg_actuator_.vel_load.resize(NUMOFSLAVES);

    msg_actuator_.joint_angle.resize(NUMOFSLAVES);
    msg_actuator_.joint_vel.resize(NUMOFSLAVES);
    msg_actuator_.current.resize(NUMOFSLAVES);
    
    msg_controller_.des_leg_pos.resize(NUMOFSLAVES);
    msg_controller_.des_leg_vel.resize(NUMOFSLAVES);
    msg_controller_.leg_pos.resize(NUMOFSLAVES);
    msg_controller_.leg_vel.resize(NUMOFSLAVES);


    const size_t N = NUMOFLEGS;  // 다리 수
    msg_gui_cmd_.kp_x.assign(N, 0.0);
    msg_gui_cmd_.ki_x.assign(N, 0.0);
    msg_gui_cmd_.kd_x.assign(N, 0.0);
    msg_gui_cmd_.d_cutoff_x.assign(N, 0.0);

    msg_gui_cmd_.kp_y.assign(N, 0.0);
    msg_gui_cmd_.ki_y.assign(N, 0.0);
    msg_gui_cmd_.kd_y.assign(N, 0.0);
    msg_gui_cmd_.d_cutoff_y.assign(N, 0.0);

    msg_gui_cmd_.kp_z.assign(N, 0.0);
    msg_gui_cmd_.ki_z.assign(N, 0.0);
    msg_gui_cmd_.kd_z.assign(N, 0.0);
    msg_gui_cmd_.d_cutoff_z.assign(N, 0.0);

  //! Init Local variable
    kp_x.assign(N, 0.0); kp_y.assign(N, 0.0); kp_z.assign(N, 0.0);
    ki_x.assign(N, 0.0); ki_y.assign(N, 0.0); ki_z.assign(N, 0.0);
    kd_x.assign(N, 0.0); kd_y.assign(N, 0.0); kd_z.assign(N, 0.0);


  //* initialize publishers *//
    auto qos_ctrl_state = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    auto qos_controller = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pub_ctrl_state_ = this->create_publisher<mcl_quadruped_msgs::msg::ControllerState>(
        "controller_data", qos_controller);
    this->initControllerState();


    auto qos_actuator = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pub_actuator_ = this->create_publisher<mcl_quadruped_msgs::msg::ActuatorState>(
        "actuator_data", qos_actuator);





  //* initialize subscribers *//

    RCLCPP_INFO(this->get_logger(), "ELMO ROS Interface Node started successfully");
    RCLCPP_INFO(this->get_logger(), "Run update loop at 100Hz");

    const auto qos_gui_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
      
  
    sub_gui_cmd = this->create_subscription<mcl_quadruped_msgs::msg::GuiCommand>(
      "gui_command", qos_gui_cmd,
      [this](const mcl_quadruped_msgs::msg::GuiCommand::SharedPtr msg) {
        this->subGuiCommand(msg);
    });

    sub_ctrl_on_ = this->create_subscription<std_msgs::msg::Bool>(
        "ctrl_on", qos_gui_cmd,
        [this](const std_msgs::msg::Bool::SharedPtr msg){
            this->ctrl_on = msg->data;
            RCLCPP_INFO(this->get_logger(), "[SUB] ctrl_on = %s", msg->data ? "true" : "false");
        });

    sub_traj_on_ = this->create_subscription<std_msgs::msg::Bool>(
        "traj_on", qos_gui_cmd,
        [this](const std_msgs::msg::Bool::SharedPtr msg){
            this->traj_on = msg->data;
            RCLCPP_INFO(this->get_logger(), "[SUB] traj_on = %s", msg->data ? "true" : "false");
        });

    sub_motor_offset_ = this->create_subscription<std_msgs::msg::Bool>(
        "motor_offset", qos_gui_cmd,
        [this](const std_msgs::msg::Bool::SharedPtr msg){
            this->motor_offset_on = msg->data;
            RCLCPP_INFO(this->get_logger(), "[SUB] motor_offset = %s", msg->data ? "true" : "false");
        });


      
  }


void ElmoRosInterface::update() {
  rclcpp::Rate loop_rate(100); // 100Hz

  RCLCPP_INFO(this->get_logger(), "Starting update loop at 100Hz...");

  while (rclcpp::ok()) {
    rclcpp::spin_some(this->shared_from_this()); // handle callback e.g. subscribers
    this->copyStateToSharedMemory();

    // TODO: Add non-RT task here if needed

    this->copyStateFromSharedMemory();
    this->pubControllerState();
    loop_rate.sleep();
  }
}


// Shared Memory의 data를 copy
void ElmoRosInterface::copyStateFromSharedMemory() {
  /* check if shared memory is connected */
  if (!shm_manager_) {
    static int error_count = 0;
    if (error_count % 100 == 0) RCLCPP_ERROR(this->get_logger(), "Shared memory not connected!");
    error_count++;
    return;
  }

  /* check system ready status */
  if (!b_system_ready_) {
    static int warn_count = 0;
    if (warn_count % 100 == 0)
      RCLCPP_WARN(this->get_logger(), "Real-time controller not ready yet...");
    warn_count++;
  }

  auto shm = shm_manager_->getSafeAccess();


  b_system_ready_ = shm->b_system_ready;

  
  const auto& a = shm->actuator;
  const auto& c = shm->control; 

    /* Add actuator data to msg*/
      msg_actuator_.stamp = this->get_clock()->now(); 


    for (int i = 0; i < NUMOFSLAVES; i++) {
      // msg_actuator_.pos_load[i] = a.pos_load[i];
      
      msg_actuator_.joint_angle[i] = a.joint_angle[i];
      msg_actuator_.current[i] = a.current[i];
      // cout << "current[" << i << "] = " << a.current[i] << endl;
      
    }

    //! Debuging
    for (int i = 0; i < NUMOFSLAVES; i++) {
      // msg_conrtoller_.leg_pos_x = c.
      msg_controller_.des_leg_pos[i] = c.des_leg_pos[i];
      msg_controller_.leg_pos[i] = c.leg_pos[i];

      // cout << "leg_pos[" << i << "] = " << c.leg_pos[i] << endl;
    }

    msg_actuator_.system_ready = shm->b_system_ready;

}

void ElmoRosInterface::copyStateToSharedMemory() {
  auto& shm = InterProcessStateManager::getInstance(); // 이미 생성된 싱글턴
  auto acc = shm.getSafeAccess();                      // lock 자동



  // auto shm = shm_manager_->getSafeAccess();


  auto& g = acc->gui_cmd; 

  //! GUI Command 
    g.ctrl_on = ctrl_on;
    g.traj_on = traj_on;
    g.motor_offset_on = motor_offset_on;   //-> 이거 어디에 쓰는거지?
    
    if(!g.kp_x.empty())
    {
      for(int i = 0; i < NUMOFLEGS; i++)
      {
        g.kp_x[i] = kp_x[i]; g.kp_y[i] = kp_y[i]; g.kp_z[i] = kp_z[i];
        g.ki_x[i] = ki_x[i]; g.ki_y[i] = ki_y[i]; g.ki_z[i] = ki_z[i];
        g.kd_x[i] = kd_x[i]; g.kd_y[i] = kd_y[i]; g.kd_z[i] = kd_z[i];
      }
    }


    // Add Process Shared Data //
    for(int i = 0; i < NUMOFSLAVES; i++)
    {
      acc->gui_cmd.controlword[i] = controlword_[i];
      
    }










  
}

void ElmoRosInterface::initControllerState() {

}

void ElmoRosInterface::pubControllerState() {

  msg_controller_.b_system_ready = b_system_ready_;

  pub_ctrl_state_->publish(msg_controller_);

  pub_actuator_->publish(msg_actuator_);


}


void ElmoRosInterface::subGuiCommand(const mcl_quadruped_msgs::msg::GuiCommand::SharedPtr msg) {
 
  // 내용 전부 출력
  std::ostringstream oss;
  oss << "[GUI] controlword = [";
  for (size_t i = 0; i < msg->controlword.size(); ++i) {
    oss << static_cast<unsigned>(msg->controlword[i]);
    if (i + 1 < msg->controlword.size()) oss << ", ";
  }
  oss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

  
  for(int i = 0; i < NUMOFSLAVES; i++)
  {
    controlword_[i] = msg->controlword[i];
  }


  if (!msg->kp_x.empty()) {
    for(int i = 0; i < NUMOFLEGS; i++)
    {
      kp_x[i] = msg->kp_x[i]; kp_y[i] = msg->kp_y[i]; kp_z[i] = msg->kp_z[i];
      ki_x[i] = msg->ki_x[i]; ki_y[i] = msg->ki_y[i]; ki_z[i] = msg->ki_z[i];
      kd_x[i] = msg->kd_x[i]; kd_y[i] = msg->kd_y[i]; kd_z[i] = msg->kd_z[i];
    }
  }


}
