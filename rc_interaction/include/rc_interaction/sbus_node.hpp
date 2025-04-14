#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <rc_interaction/basic_debugger.hpp>
#include <rc_interaction/msg/sbus.hpp>
#include <rc_interaction/protocols/sbus_serial.hpp>
#include <rc_interaction/utils.hpp>

#include <algorithm>
#include <boost/algorithm/clamp.hpp>

namespace robot {
class sbus_node : public rclcpp::Node {
public:
  using PointT = geometry_msgs::msg::Point;
  using VelCovT = geometry_msgs::msg::TwistWithCovariance;
  using VelT = geometry_msgs::msg::TwistStamped;
  using CmdT = std_msgs::msg::Int8;
  using PoseStampT = geometry_msgs::msg::PoseStamped;
  using PoseT = geometry_msgs::msg::Pose;
  using SbusT = rc_interaction::msg::Sbus;

  enum StateT { MOV, SHOOT, NONE };//命令，移动/射击/无

private:
  std::string port_name_;
  std::string sbus_topic_;
  std::string pose_topic_;
  std::string vel_topic_;
  std::string cmd_topic_;
  sbus_serial::SBusSerialPort *port_;
  SbusT sbus_msg_;
  int rx_min_;
  int rx_max_;
  int out_min_;
  int out_max_;
  int deadband_;
  bool silent_failsafe_;
  int channel_;
  double proportional_min_;
  double proportional_max_;
  float raw_span_;
  float out_span_;

  rclcpp::Time last_published_timestamp_;

  PoseT pose_;
  VelT vel_;
  StateT state_ = NONE;
  CmdT cmd_;

  const tf2::Vector3 basket_vec_{0, 0, 0};//篮筐坐标

  rclcpp::Publisher<CmdT>::SharedPtr cmd_pub_;
  rclcpp::Publisher<VelT>::SharedPtr vel_pub_;

  rclcpp::Subscription<PoseStampT>::SharedPtr pose_sub_;
  rclcpp::Publisher<SbusT>::SharedPtr sbus_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  sbus_node() : Node("sbus_node") {

    this->declare_parameter("port", port_name_);
    this->declare_parameter("sbus_topic", sbus_topic_);
    this->declare_parameter("pose_topic", pose_topic_);
    this->declare_parameter("vel_topic", vel_topic_);
    this->declare_parameter("cmd_topic", cmd_topic_);
    this->declare_parameter("rx_min", rx_min_);
    this->declare_parameter("rx_max", rx_max_);
    this->declare_parameter("out_min", out_min_);
    this->declare_parameter("out_max", out_max_);
    this->declare_parameter("deadband", deadband_);
    this->declare_parameter("silent_failsafe", silent_failsafe_);
    this->declare_parameter("channel", channel_);
    this->declare_parameter("proportional_min_", proportional_min_);
    this->declare_parameter("proportional_max", proportional_max_);
    this->declare_parameter("refresh_hz", 2);

    int refresh_hz_;

    GET_PARAM_DEBUG("port", port_name_);
    GET_PARAM_DEBUG("sbus_topic", sbus_topic_);
    GET_PARAM_DEBUG("pose_topic", pose_topic_);
    GET_PARAM_DEBUG("vel_topic", vel_topic_);
    GET_PARAM_DEBUG("cmd_topic", cmd_topic_);
    GET_PARAM_DEBUG("rx_min", rx_min_);
    GET_PARAM_DEBUG("rx_max", rx_max_);
    GET_PARAM_DEBUG("out_min", out_min_);
    GET_PARAM_DEBUG("out_max", out_max_);
    GET_PARAM_DEBUG("deadband", deadband_);
    GET_PARAM_DEBUG("silent_failsafe", silent_failsafe_);
    GET_PARAM_DEBUG("channel", channel_);
    GET_PARAM_DEBUG("proportional_min_", proportional_min_);
    GET_PARAM_DEBUG("proportional_max", proportional_max_);
    GET_PARAM_DEBUG("refresh_hz", refresh_hz_);

    raw_span_ = static_cast<float>(rx_max_ - rx_min_);
    out_span_ = static_cast<float>(out_max_ - out_min_);

    cmd_pub_ = this->create_publisher<CmdT>(cmd_topic_, 10);
    vel_pub_ = this->create_publisher<VelT>(vel_topic_, 10);
    sbus_pub_ = this->create_publisher<SbusT>(sbus_topic_, 10);

    pose_sub_ = this->create_subscription<PoseStampT>(
        pose_topic_, 10,
        std::bind(&sbus_node::pose_feedback, this, std::placeholders::_1));

    try {
      port_ = new sbus_serial::SBusSerialPort(port_name_, true);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unable to initalize SBUS port");
      throw;
    }

    sbus_msg_ = rc_interaction::msg::Sbus();

    auto callback = [&](const sbus_serial::SBusMsg msg_) {
      constexpr auto MIDDLE = 1000;
      constexpr auto SPEED_MAX = 2.0f;
      constexpr auto ANGULAR_MAX = 1.0f;

      if (silent_failsafe_ && msg_.failsafe)
        return;
      if (msg_.frame_lost)
        return;
      // vel
      vel_.twist.linear.x = (msg_.channels[2] - MIDDLE) * 100.0f / 666;
      vel_.twist.linear.x = SPEED_MAX * vel_.twist.linear.x / 100;

      vel_.twist.linear.y = (msg_.channels[3] - MIDDLE) * 100.0f / 666;
      vel_.twist.linear.y = SPEED_MAX * vel_.twist.linear.y / 100;

      vel_.twist.angular.z = (msg_.channels[0] - MIDDLE) * 100.0f / 666;
      vel_.twist.angular.z = ANGULAR_MAX * vel_.twist.angular.z / 100;

      vel_.header.stamp = this->get_clock()->now();
      vel_.header.frame_id = "";
      // cmd button
      // TODO
      cmd_.data = state_;
      // sbus msg
      // TODO
    };

    port_->setCallback(callback);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000 / refresh_hz_),
                                std::bind(&sbus_node::timer_callback, this));
    RCLCPP_INFO(this->get_logger(),
                "SBUS node started, publisher created, timer scheduled...");
  }

private:
  void pose_feedback(const PoseStampT::ConstSharedPtr &msg) {

    pose_ = msg->pose;
    tf2::Quaternion pose_quat_(pose_.orientation.x, pose_.orientation.y,
                               pose_.orientation.z, pose_.orientation.w);
    tf2::Matrix3x3 m(pose_quat_);
    double r, p, y;
    m.getRPY(r, p, y);

    //右上是零点,向左x递增向下y递增,y轴的负方向上yaw=0
    // yaw 逆时针为正,angle 顺时针为正
    /**
    <---*----------
     | /\ |       |
     |/__\|       |
                  |
                  |
                  |
                  ↓
     */
    double angle =
        std::atan2((pose_.position.x - basket_vec_.x()), pose_.position.y);
    ROBOT_INFO_F("desire yaw: %f ; current yaw: %f ; robot turn %s", angle, y,
                 angle > y ? "right" : "left");
    double delta_angle;
    if(angle>0 && y>0){
      //顺时针  负方向转
    }
    else if(angle<0 && y<0)
    {
        //逆时针  正方向转
    }else if(y>0){
        //都在左边 angle>0
        y+angle>0;//顺时针
    }else
    { 
       //都在右边  y<0
       angle+y<0;//逆时针
    }

    // TODO
    // calculate yaw speed
    // vel_.twist.angular.x;
    return;
  }

  void timer_callback() {

    if (last_published_timestamp_ != vel_.header.stamp) {
      sbus_pub_->publish(sbus_msg_);
      vel_pub_->publish(vel_);
      cmd_pub_->publish(cmd_);
      last_published_timestamp_ = vel_.header.stamp;
    }
  }
};
} // namespace robot