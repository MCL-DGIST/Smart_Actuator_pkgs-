#include "mcl_quadruped_gui/mcl_quadruped_gui.hpp"

#include <pluginlib/class_list_macros.hpp>

/* Qt libraries*/
#include <QStringList>

/* C++ STL */
#include <bitset>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

using std::placeholders::_1;



namespace mcl_quadruped_gui
{
/**
 * Constructor is called first before initPlugin function, needless to say.
 */
MclQuadrupedGui::MclQuadrupedGui() : rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("MclQuadrupedGui");  // give QObjects reasonable names

  node_ = std::make_shared<rclcpp::Node>("mcl_quadruped_gui");



  // TODO: Define QoS, Publishers, and Subscribers here
  
  



  // rclcpp::QoS(rclcpp::KeepLast(10)) 버퍼를 10개까지만 쌓아준다는 뜻. 넘으면 오래된것 부터 버림
  const auto qos_gui = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  pub_gui_cmd_ = node_->create_publisher<mcl_quadruped_msgs::msg::GuiCommand>("gui_command", qos_gui);
  pub_ctrl_on_  = node_->create_publisher<std_msgs::msg::Bool>("ctrl_on", qos_gui);
  pub_traj_on_  = node_->create_publisher<std_msgs::msg::Bool>("traj_on", qos_gui);
  pub_motor_offset_ = node_->create_publisher<std_msgs::msg::Bool>("motor_offset", qos_gui);







// motor_offset




  // ✅ ActuatorState 구독자 추가
  sub_actuator_ = node_->create_subscription<mcl_quadruped_msgs::msg::ActuatorState>(
      "actuator_data", 10,
      [this](const mcl_quadruped_msgs::msg::ActuatorState::SharedPtr msg){

      for(int i = 0; i < NUMOFLEGS_gui; i++)
      { 
        joint_angle[i] = msg->joint_angle[i];
        current[i] = msg->current[i];
        cout << "joint_angle[" << i << "] = " << joint_angle[i] << endl;
      }
  });

  sub_controller_ = node_->create_subscription<mcl_quadruped_msgs::msg::ControllerState>(
      "controller_data", 10,
      [this](const mcl_quadruped_msgs::msg::ControllerState::SharedPtr msg){

      // for(int i = 0; i < 2; i++)
      // { 
      //   leg_pos[i] = msg->leg_pos[i];
      //   des_leg_pos[i] = msg->des_leg_pos[i];
      //   cout << "leg_pos[" << i << "] = " << leg_pos[i] << endl;
      // }
  });



  /* ROS Node Spinning */
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() {
    rclcpp::spin_some(node_);
    this->pubGuiCommand();
  });
  spin_timer_->start(SPIN_PERIOD_MS);
}

//* ----- PLUGIN INTERFACE -----------------------------------------------------------------------
void MclQuadrupedGui::initPlugin(qt_gui_cpp::PluginContext & context)
{
  QStringList argv = context.argv();  // access standalone command line arguments
  widget_ = new QWidget();            // create QWidget
  ui_.setupUi(widget_);

    
  // TODO: Set widget properties & Define SIGNAL & SLOT connections here
  

  QString styleSheet =
    "QCheckBox::indicator { width:50px; height:50px; }"
    "QCheckBox::indicator:checked { image: url(:/icons/on-button.png); }"
    "QCheckBox::indicator:unchecked { image: url(:/icons/off-button.png); }";


  timeTicker.reset(new QCPAxisTickerTime());  
  timeTicker->setTimeFormat("%m:%s");

  //! Create Plot Start !//

    createPlot(ui_.Motor_ang);
    createPlot(ui_.Motor_vel);
    // createPlot(ui_.FL_pos_x);
    // createPlot(ui_.FL_pos_z);
    createPlot(ui_.Motor_current);
    // createPlot(ui_.FLKNEE_current);    

  
  context.addWidget(widget_);  // add widget to user interface

  ui_ready_ = true;
  
  ctrlword_state_.assign(NUMOFSLAVES_gui, 0);

  //! Enable Motor !//
      connect(ui_.enable_FLHIP, &QPushButton::toggled, this, [this](bool on){
      if (on) {send_ctrl_word_sequence(0);} 
      else {
        ctrlword_state_[0] = 0;
        publish_control_word();
      }
      });

      // connect(ui_.enable_FLKNEE, &QPushButton::toggled, this, [this](bool on){
      // if (on) {send_ctrl_word_sequence(1);} 
      // else {
      //   ctrlword_state_[1] = 0;
      //   publish_control_word();
      // }
      // });
  
  //! On/Off Switch !//
      connect(ui_.ctrl_on, &QPushButton::toggled, this, [this](bool on){
          std_msgs::msg::Bool msg;
          msg.data = on;
          pub_ctrl_on_->publish(msg);

          RCLCPP_INFO(node_->get_logger(), "[GUI] publish ctrl_on = %s", on ? "true" : "false");
      });

      
      // traj_on 버튼 눌렀을 때
      connect(ui_.traj_on, &QPushButton::toggled, this, [this](bool on){
          std_msgs::msg::Bool msg;
          msg.data = on;
          pub_traj_on_->publish(msg);

          RCLCPP_INFO(node_->get_logger(), "[GUI] publish traj_on = %s", on ? "true" : "false");
      });

      connect(ui_.motor_offset, &QPushButton::toggled, this, [this](bool on){
          std_msgs::msg::Bool msg;
          msg.data = on;
          pub_motor_offset_->publish(msg);

          RCLCPP_INFO(node_->get_logger(), "[GUI] publish motor_offset = %s", on ? "true" : "false");
      });

  //! Parameter Set Button !//
      connect(ui_.Set, &QPushButton::clicked, this, [this](bool on){
        mcl_quadruped_msgs::msg::GuiCommand g;

        g.controlword = ctrlword_state_;
        
        const size_t N = NUMOFLEGS_gui;
        g.kp_x.resize(N); g.ki_x.resize(N); g.kd_x.resize(N); g.d_cutoff_x.resize(N);
        g.kp_y.resize(N); g.ki_y.resize(N); g.kd_y.resize(N); g.d_cutoff_y.resize(N);
        g.kp_z.resize(N); g.ki_z.resize(N); g.kd_z.resize(N); g.d_cutoff_z.resize(N);
        // 필요하면 y도:
        // g.Kp_y.resize(N); g.Ki_y.resize(N); g.Kd_y.resize(N); g.d_cutoff_y.resize(N);

        // 현재 UI 값 한 번만 캡처해서 [0]에 대입 (value() 호출!)
        g.kp_x[0]       = ui_.FL_x_kp->value();
        g.ki_x[0]       = ui_.FL_x_ki->value();
        g.kd_x[0]       = ui_.FL_x_kd->value();
        g.d_cutoff_x[0] = ui_.FL_x_d_cutoff->value();

        g.kp_y[0]       = ui_.FL_y_kp->value();
        g.ki_y[0]       = ui_.FL_y_ki->value();
        g.kd_y[0]       = ui_.FL_y_kd->value();
        g.d_cutoff_y[0] = ui_.FL_y_d_cutoff->value();

        g.kp_z[0]       = ui_.FL_z_kp->value();
        g.ki_z[0]       = ui_.FL_z_ki->value();
        g.kd_z[0]       = ui_.FL_z_kd->value();
        g.d_cutoff_z[0] = ui_.FL_z_d_cutoff->value();
        
        
        RCLCPP_INFO(node_->get_logger(),
        "[GUI] Set -> X(Kp,Ki,Kd,cut)=%.3f,%.3f,%.3f,%.3f | Z=%.3f,%.3f,%.3f,%.3f",
        g.kp_x[0], g.ki_x[0], g.kd_x[0], g.d_cutoff_x[0],
        g.kp_z[0], g.ki_z[0], g.kd_z[0], g.d_cutoff_z[0]);

        pub_gui_cmd_->publish(g);

    });
      
  // 50ms마다 캐시 -> 라벨 반영
  gui_update_timer_ = new QTimer(this);
    connect(gui_update_timer_, &QTimer::timeout, [this](){
    if (!ui_ready_) return;


    update_ui();
    


  });
  gui_update_timer_->start(10);

}

void MclQuadrupedGui::publish_control_word() {
  mcl_quadruped_msgs::msg::GuiCommand g;
  g.controlword = ctrlword_state_;

  std::ostringstream oss;
  oss << "[GUI] publish controlword = [";
  for (size_t i = 0; i < ctrlword_state_.size(); ++i) {
    oss << ctrlword_state_[i];
    if (i + 1 < ctrlword_state_.size()) oss << ", ";
  }
  oss << "]";
  RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());

  pub_gui_cmd_->publish(g);
}

void MclQuadrupedGui::update_ui()
{
  static QTime timeStart = QTime::currentTime();          // calculate two new data points:
  key = timeStart.msecsTo(QTime::currentTime()) / 1000.0; // time elapsed since start of demo, in seconds

  ui_.pos_hip->setText(QString::number(joint_angle[0], 'f', 3));
  // ui_.pos_knee->setText(QString::number(joint_angle[1], 'f', 3));

  drawPlot(ui_.Motor_ang, joint_angle[0], joint_angle[0]);
  drawPlot(ui_.Motor_vel, joint_vel[0], joint_vel[0]);
  // drawPlot(ui_.FL_pos_x, leg_pos[0], des_leg_pos[0]);
  // drawPlot(ui_.FL_pos_z, leg_pos[1], des_leg_pos[1]);
  drawPlot(ui_.Motor_current, current[0], current[0]);
  // drawPlot(ui_.FLKNEE_current, current[1], current[1]);
  


}

void MclQuadrupedGui::send_ctrl_word_sequence(int motor_index)
{
  if (motor_index < 0 || motor_index >= motor_count_) {
    RCLCPP_WARN(node_->get_logger(), "Invalid motor index: %d", motor_index);
    return;
  }



auto set_and_pub = [this, motor_index](uint16_t v){
  ctrlword_state_[static_cast<size_t>(motor_index)] = v;
  RCLCPP_INFO(node_->get_logger(),
              "[GUI] motor %d set %u -> publish", motor_index, (unsigned)v);
  publish_control_word();
};
  // 0ms: 128 → 50ms: 6 → 100ms: 15
  set_and_pub(128);
  QTimer::singleShot(50,  this, [set_and_pub](){ set_and_pub(6);  });
  QTimer::singleShot(100, this, [set_and_pub](){ set_and_pub(15); });
}
/**
 * @note Add objects that are required to be reset
 */
void MclQuadrupedGui::shutdownPlugin()
{
  if (gui_update_timer_)
  {
    gui_update_timer_->stop();
    delete gui_update_timer_;
    gui_update_timer_ = nullptr;
  }

  if (spin_timer_)
  {
    spin_timer_->stop();
    delete spin_timer_;
    spin_timer_ = nullptr;
  }
}

void MclQuadrupedGui::saveSettings(qt_gui_cpp::Settings & plugin_settings,
                                   qt_gui_cpp::Settings & instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v);
}

void MclQuadrupedGui::restoreSettings(const qt_gui_cpp::Settings & plugin_settings,
                                      const qt_gui_cpp::Settings & instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k);
}

void MclQuadrupedGui::pubGuiCommand()
{
  // auto msg = std_msgs::msg::Bool();

  


  // pub_gui_cmd_->publish(msg);
  

  

}

  void MclQuadrupedGui::createPlot(QCustomPlot *plot) {
    plot->addGraph();
    plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
    plot->addGraph();
    plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
    plot->xAxis->setTicker(timeTicker);
    plot->axisRect()->setupFullAxesBox();
    connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
    plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);

    plot->xAxis->setBasePen(QPen(Qt::white, 1));
    plot->yAxis->setBasePen(QPen(Qt::white, 1));
    plot->xAxis->setTickPen(QPen(Qt::white, 1));
    plot->yAxis->setTickPen(QPen(Qt::white, 1));
    plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
    plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
    plot->xAxis->setTickLabelColor(Qt::white);
    plot->yAxis->setTickLabelColor(Qt::white);
    plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    plot->xAxis->grid()->setSubGridVisible(true);
    plot->yAxis->grid()->setSubGridVisible(true);
    plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
    plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
    plot->setBackground(QColor(25, 35, 45));
    plot->axisRect()->setBackground(QColor(25, 35, 45));
  }


void MclQuadrupedGui::drawPlot(QCustomPlot *plot, double data1, double data2){
  
  plot->graph(0)->addData(key,data1);
  plot->graph(1)->addData(key,data2);
  plot->xAxis->setRange(key,Plot_time_window_POS,Qt::AlignRight);


    plot->yAxis->rescale();
  
  
  plot->graph(0)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(1)->data()->removeBefore(key-Plot_time_window_POS);
  plot->replot();

}


}  // namespace mcl_quadruped_gui





PLUGINLIB_EXPORT_CLASS(mcl_quadruped_gui::MclQuadrupedGui, rqt_gui_cpp::Plugin)