#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ros2_socketcan/socket_can_id.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <ros2_socketcan/visibility_control.hpp>
#include <ros2_socketcan_msgs/msg/fd_frame.hpp>

#include <rc_planning/basic_debugger.hpp>
#include <rc_planning/ekf_config.hpp>
#include <rc_planning/quaternion_solver.hpp>
#include <rc_planning/utils.hpp>

namespace robot {

using namespace drivers::socketcan;

enum State { MOV, SHOOT };

class planning_node : public rclcpp::Node {

public:
  using OdomT = nav_msgs::msg::Odometry;
  using ImuT = sensor_msgs::msg::Imu;
  using FrameT = ros2_socketcan_msgs::msg::FdFrame;
  using PointT = geometry_msgs::msg::Point;
  using AccelT = geometry_msgs::msg::Accel;
  using VelCovT = geometry_msgs::msg::TwistWithCovariance;
  using VelT = geometry_msgs::msg::Twist;
  using CmdT = std_msgs::msg::Int8;

  explicit planning_node() : Node("planning_node") {

    // construct
    this->declare_parameter("receiver_interface", "can0");
    this->declare_parameter("sender_interface", "can0");
    this->declare_parameter<bool>("use_bus_time", false);
    this->declare_parameter("interval_sec", 0.01);
    this->declare_parameter("timeout_sec", 0.01);
    this->declare_parameter("filters", "0:0");
    this->declare_parameter("odom_frame_id", "odom_combined");
    this->declare_parameter("robot_frame_id", "base_footprint");
    this->declare_parameter("imu_frame_id", "gyro_link");

    GET_PARAM_DEBUG("receiver_interface", receiver_interface_);
    GET_PARAM_DEBUG("sender_interface", sender_interface_);
    GET_PARAM_DEBUG("use_bus_time", use_bus_time_);
    GET_PARAM_DEBUG("filters", filters_);
    GET_PARAM_DEBUG("interval_sec", interval_s_);
    GET_PARAM_DEBUG("timeout_sec", timeout_s_);
    GET_PARAM_DEBUG("odom_frame_id", odom_frame_id);
    GET_PARAM_DEBUG("robot_frame_id", robot_frame_id);
    GET_PARAM_DEBUG("imu_frame_id", imu_frame_id);

    interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(interval_s_));
    timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_s_));

    // configure
    odom_pub_ = this->create_publisher<OdomT>("/odom", 2);
    fd_frames_pub_ = this->create_publisher<FrameT>("fd_frame", 500);
    imu_pub_ = this->create_publisher<ImuT>("imu/data_raw", 2);
    vel_sub_ =
        create_subscription<VelT>("cmd_vel", 2,
                                  std::bind(&planning_node::cmd_vel_callback,
                                            this, std::placeholders::_1));
    cmd_sub_ = create_subscription<CmdT>(
        "cmd_behav", 2,
        std::bind(&planning_node::cmd_callback, this, std::placeholders::_1));

    CanId receive_id{};
    ros2_socketcan_msgs::msg::FdFrame fd_frame_msg(
        rosidl_runtime_cpp::MessageInitialization::ZERO);
    fd_frame_msg.header.frame_id = "can";

    try {
      receiver_ =
          std::make_unique<SocketCanReceiver>(receiver_interface_, true);
      receiver_->SetCanFilters(SocketCanReceiver::CanFilterList(filters_));
    } catch (const std::exception &ex) {
      ROBOT_ERROR_F("Error opening CAN receiver: %s - %s",
                    receiver_interface_.c_str(), ex.what());
      exit(-1);
    }
    ROBOT_INFO("Receiver successfully configured.");
    try {
      sender_ = std::make_unique<SocketCanSender>(sender_interface_, true);
    } catch (const std::exception &ex) {
      ROBOT_ERROR_F("Error opening CAN sender: %s - %s",
                    sender_interface_.c_str(), ex.what());
      exit(-1);
    }
    ROBOT_INFO("Sender successfully configured.");

    // activate
    while (rclcpp::ok()) {

      now_ = this->now();
      sample_time_ = (now_ - last_).seconds();

      fd_frame_msg.data.resize(64);

      try {
        receive_id =
            receiver_->receive_fd(fd_frame_msg.data.data<void>(), interval_ns_);
      } catch (const std::exception &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Error receiving CAN FD message: %s - %s",
                             receiver_interface_.c_str(), ex.what());
        continue;
      }

      fd_frame_msg.data.resize(receive_id.length());
      if (use_bus_time_) {
        fd_frame_msg.header.stamp = rclcpp::Time(
            static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
      } else {
        fd_frame_msg.header.stamp = this->now();
      }

      fd_frame_msg.id = receive_id.identifier();
      fd_frame_msg.is_extended = receive_id.is_extended();
      fd_frame_msg.is_error = (receive_id.frame_type() == FrameType::ERROR);
      fd_frame_msg.len = receive_id.length();
      fd_frames_pub_->publish(std::move(fd_frame_msg));

      velocity_.twist.linear.x = 0;
      velocity_.twist.linear.y = 0;
      velocity_.twist.linear.z = 0;
      velocity_.twist.angular.x = 0;
      velocity_.twist.angular.y = 0;
      velocity_.twist.angular.z = 0;

      accel_.linear.x = 0;
      accel_.linear.y = 0;
      accel_.linear.z = 0;
      // accel_.angular.x = 0;
      // accel_.angular.y = 0;
      // accel_.angular.z = 0;
      // TODO

      pos_.x += (velocity_.twist.linear.x * cos(pos_.z) -
                 velocity_.twist.linear.y * sin(pos_.z)) *
                sample_time_;
      pos_.y += (velocity_.twist.linear.x * sin(pos_.z) -
                 velocity_.twist.linear.y * cos(pos_.z)) *
                sample_time_;
      pos_.z += velocity_.twist.linear.z * sample_time_;

      detail::quaternion_solver(velocity_.twist.angular.x,
                                velocity_.twist.angular.y,
                                velocity_.twist.angular.z, accel_.linear.x,
                                accel_.linear.y, accel_.linear.z, imu_);
      imu_.header.stamp = this->now();
      imu_.header.frame_id = imu_frame_id;
      imu_.orientation_covariance[0] =
          1e6; // Three-axis attitude covariance matrix
      imu_.orientation_covariance[4] = 1e6;
      imu_.orientation_covariance[8] = 1e-6;
      imu_.angular_velocity = velocity_.twist.angular;
      imu_.angular_velocity_covariance[0] =
          1e6; // Triaxial angular velocity covariance matrix
      imu_.angular_velocity_covariance[4] = 1e6;
      imu_.angular_velocity_covariance[8] = 1e-6;
      imu_.linear_acceleration = accel_.linear;
      imu_pub_->publish(imu_);

      tf2::Quaternion q;
      q.setRPY(0, 0, pos_.z);
      odom_.pose.pose.orientation = tf2::toMsg(q);
      odom_.header.stamp = rclcpp::Node::now();
      odom_.header.frame_id = odom_frame_id;
      odom_.pose.pose.position = pos_;
      odom_.child_frame_id = robot_frame_id;
      odom_.twist = velocity_;
      if (velocity_.twist.linear.x == 0 && velocity_.twist.linear.y == 0 &&
          velocity_.twist.linear.z == 0) {
        odom_.twist.covariance = ekf_config::odom_twist_covariance2;
        odom_.pose.covariance = ekf_config::odom_pose_covariance2;
      } else {
        odom_.twist.covariance = ekf_config::odom_twist_covariance;
        odom_.pose.covariance = ekf_config::odom_pose_covariance;
      }
      odom_pub_->publish(odom_);

      last_ = now_;

      rclcpp::spin_some(this->get_node_base_interface());
    }
  }

private:
  void cmd_vel_callback(const VelT::ConstSharedPtr &msg_) {

    msg_->angular.x;
    FrameT frame_fd_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    frame_fd_msg.header.frame_id = "can";
    frame_fd_msg.id = 1145;
    frame_fd_msg.is_error = false;
    frame_fd_msg.is_extended = true;
    // msg.len=
    // TODOTODO

    CanId send_id = CanId(frame_fd_msg.id, 0, FrameType::DATA, ExtendedFrame);
    try {
      sender_->send_fd(frame_fd_msg.data.data<void>(), frame_fd_msg.len,
                       send_id, timeout_ns_);
    } catch (const std::exception &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Error sending CAN message: %s - %s",
                           sender_interface_.c_str(), ex.what());
      return;
    }
  }

  void cmd_callback(const CmdT::ConstSharedPtr &msg_) { cmd_ = msg_->data; }

  std::string odom_frame_id;
  std::string robot_frame_id;
  std::string imu_frame_id;

  rclcpp::Publisher<OdomT>::SharedPtr odom_pub_;
  rclcpp::Publisher<ImuT>::SharedPtr imu_pub_;
  rclcpp::Publisher<FrameT>::SharedPtr fd_frames_pub_;

  rclcpp::Subscription<VelT>::SharedPtr vel_sub_;
  rclcpp::Subscription<CmdT>::SharedPtr cmd_sub_;

  rclcpp::Time now_, last_;
  float sample_time_;

  VelCovT velocity_;
  PointT pos_;
  AccelT accel_;
  OdomT odom_;
  ImuT imu_;
  CmdT::_data_type cmd_;

  double interval_s_;
  std::chrono::nanoseconds interval_ns_;
  bool use_bus_time_;
  std::string receiver_interface_;
  std::string filters_;
  std::unique_ptr<SocketCanReceiver> receiver_;

  double timeout_s_;
  std::chrono::nanoseconds timeout_ns_;
  std::string sender_interface_;
  std::unique_ptr<SocketCanSender> sender_;
};
} // namespace robot

/**
while(ok){
    receive_can();
    get_sender_data();
    send_can();
}
 */