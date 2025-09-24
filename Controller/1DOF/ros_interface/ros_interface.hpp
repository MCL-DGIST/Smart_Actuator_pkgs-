
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include "mcl_quadruped_msgs/msg/actuator_state.hpp"
#include "mcl_quadruped_msgs/msg/controller_state.hpp"
#include "mcl_quadruped_msgs/msg/gui_command.hpp"
#include <std_msgs/msg/float64.hpp>

/* C/C++ STL */
#include <chrono>
#include <memory>
#include <vector>
#include <atomic>
#include <array>

/* Custom libraries */
#include "ecat_device_control_objects.hpp"
#include "inter_process_state_manager.hpp"
#include "process_shared_data.hpp"

using namespace std::chrono_literals;

class ElmoRosInterface : public rclcpp::Node {
private:
  //* Publishers *//
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    rclcpp::Publisher<mcl_quadruped_msgs::msg::ControllerState>::SharedPtr pub_ctrl_state_;
    void initControllerState();
    void pubControllerState();

  //* Subscribers *//
    rclcpp::Subscription<mcl_quadruped_msgs::msg::GuiCommand>::SharedPtr sub_gui_cmd;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ctrl_on_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_traj_on_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motor_offset_;

    void subGuiCommand(const mcl_quadruped_msgs::msg::GuiCommand::SharedPtr msg);





  ElmoPtwiDriver elmo_[SLAVES_NUM];

  /* Read from shared memory (Publish to GUI) */
  EthercatStatus ecat_;

  bool b_system_ready_;


  //! Actuator State !//
    rclcpp::Publisher<mcl_quadruped_msgs::msg::ActuatorState>::SharedPtr pub_actuator_;
    mcl_quadruped_msgs::msg::ActuatorState msg_actuator_;
    
  //! Controller State !//
    // mcl_quadruped_msgs::msg::ControllerState msg_ctrl_state_;
    mcl_quadruped_msgs::msg::ControllerState msg_controller_;

  //! Gui Command !//
    mcl_quadruped_msgs::msg::GuiCommand msg_gui_cmd_;
    std::array<uint16_t, NUMOFSLAVES> controlword_{};
    bool ctrl_on = false;
    bool traj_on = false;
    bool motor_offset_on = false;


    //* Gain
    std::vector<double> kp_x, kp_y, kp_z;
    std::vector<double> ki_x, ki_y, ki_z;
    std::vector<double> kd_x, kd_y, kd_z;
    std::vector<double> d_cutoff_x, d_cutoff_y, d_cutoff_z;

    
  /* Shared memory */
  InterProcessStateManager *shm_manager_; // inter-process communication (multi-processing)
    void copyStateFromSharedMemory();
    void copyStateToSharedMemory();
  // /* Write to shared memory (Subscribe from GUI)*/
  // double joint_tau_target_[SLAVES_NUM] = {0.0};



public:
  ElmoRosInterface();

  void update();
};
