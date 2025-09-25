/** @file mcl_quadruped_gui.hpp
 * ! If you declare function in a header, while not defining it in source file, linking error occurs
 * * It is better to use `virtual` keyword for protected slot function
 */

#ifndef MCL_QUADRUPED_GUI_HPP_
#define MCL_QUADRUPED_GUI_HPP_

/* C++ STL */
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

/* ROS2 packages */
#include <rqt_gui_cpp/rqt_gui_cpp/plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>


#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
/* UI widget header */
#include <ui_mcl_quadruped_gui.h>

/* Qt libraries */
#include <qcustomplot.h>
#include <QPair>
#include <QTimer>
#include <QVector>
#include <QWidget>



#include "mcl_quadruped_msgs/msg/actuator_state.hpp"
#include "mcl_quadruped_msgs/msg/controller_state.hpp"
#include "mcl_quadruped_msgs/msg/gui_command.hpp"

#define NUMOFSLAVES_gui 1
#define NUMOFLEGS_gui 1

namespace mcl_quadruped_gui
{
class MclQuadrupedGui : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

private:
  Ui::MclQuadrupedGui ui_;  // ! don't forget to change as class name
  QWidget * widget_;

  QTimer * spin_timer_ = nullptr;
  const int SPIN_PERIOD_MS = 10;  // subscribe & publish at 10ms

  QTimer * gui_update_timer_ = nullptr;
  const int GUI_UPDATE_PERIOD_MS = 15;  // approximate human's reaction time (60Hz = 16.7ms)
  
  bool ui_ready_{false};

  //* ----- ROS INTERFACE --------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;

  //* == Publisher == *//
    rclcpp::Publisher<mcl_quadruped_msgs::msg::GuiCommand>::SharedPtr pub_gui_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ctrl_on_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_traj_on_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motor_offset_;


    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motor_offset_;

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_motor_offset_;

    std::vector<uint16_t> ctrlword_state_; 
  
  

  //* == Subscriber == *//
    rclcpp::Subscription<mcl_quadruped_msgs::msg::ActuatorState>::SharedPtr sub_actuator_;
    rclcpp::Subscription<mcl_quadruped_msgs::msg::ControllerState>::SharedPtr sub_controller_;


  QSharedPointer<QCPAxisTickerTime> timeTicker;

  // 모터 개수(예: FLHIP=0, FRHIP=1)
  int motor_count_{NUMOFSLAVES_gui};


  //! Actuator State !//
    std::vector<double> joint_angle = std::vector<double>(NUMOFSLAVES_gui, 0.0);
    std::vector<double> joint_vel = std::vector<double>(NUMOFSLAVES_gui, 0.0);
    std::vector<double> current = std::vector<double>(NUMOFSLAVES_gui, 0.0);
    
  //! Leg State !//
    // std::vector<double> leg_pos = std::vector<double>(NUMOFSLAVES_gui, 0.0);
    // std::vector<double> des_leg_pos = std::vector<double>(NUMOFSLAVES_gui, 0.0);
  
  //! GUI Command !//
  std::vector<double> kp_x, kp_y, kp_z;
  std::vector<double> ki_x, ki_y, ki_z;
  std::vector<double> kd_x, kd_y, kd_z;
  std::vector<double> d_cutoff_x, d_cutoff_y, d_cutoff_z;


  double key;
  double Plot_time_window_POS = 1.0;
    
  void pubGuiCommand();


public:
  MclQuadrupedGui();

  //* ----- PLUGIN INTERFACE -----------------------------------------------------------------------
  // ! Don't change these functions
  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings & plugin_settings,
                            qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings & plugin_settings,
                               const qt_gui_cpp::Settings & instance_settings);

  
  void update_ui();
  void publish_control_word();
  void send_ctrl_word_sequence(int motor_index);
  void createPlot(QCustomPlot *plot);
  void drawPlot(QCustomPlot *plot, double data1, double data2);
  // void send_ctrl_word_sequence();
  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

protected slots:
};
};  // namespace mcl_quadruped_gui

#endif  // MCL_QUADRUPED_GUI_HPP_