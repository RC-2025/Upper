/**
 * @file sbus_node.hpp
 * @brief SBUS遥控器节点类定义
 * 
 * 该文件定义了用于处理SBUS遥控器信号的ROS2节点类
 * 主要功能包括：
 * - 通过串口接收SBUS信号
 * - 解析遥控器通道数据
 * - 转换为机器人控制命令
 * - 发布速度指令和状态信息
 */

#pragma once
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
using namespace std;
namespace robot {

/**
 * @class sbus_node
 * @brief SBUS遥控器处理节点
 * 
 * 该类继承自rclcpp::Node，负责：
 * - 初始化SBUS串口通信
 * - 解析遥控器信号
 * - 发布控制命令
 * - 处理机器人姿态反馈
 */
class sbus_node : public rclcpp::Node {
public:
  /// 类型别名定义
  using PointT = geometry_msgs::msg::Point;          ///< 点坐标类型
  using VelCovT = geometry_msgs::msg::TwistWithCovariance; ///< 带协方差的速度类型
  using VelT = geometry_msgs::msg::TwistStamped;     ///< 带时间戳的速度类型
  using CmdT = std_msgs::msg::Int8;                  ///< 控制命令类型
  using PoseStampT = geometry_msgs::msg::PoseStamped;///< 带时间戳的位姿类型
  using PoseT = geometry_msgs::msg::Pose;            ///< 位姿类型
  using SbusT = rc_interaction::msg::Sbus;           ///< SBUS消息类型

  /// 机器人状态枚举
  enum StateT { 
    MOV,    ///< 移动状态
    SHOOT,  ///< 射击状态
    NONE    ///< 无状态
  };

private:
  // 配置参数
  std::string port_name_;       ///< SBUS串口设备路径
  std::string sbus_topic_;      ///< SBUS原始数据发布话题
  std::string pose_topic_;      ///< 位姿订阅话题(来自定位模块)
  std::string vel_topic_;       ///< 速度命令发布话题
  std::string cmd_topic_;       ///< 控制命令发布话题
  
  // SBUS相关
  sbus_serial::SBusSerialPort *port_; ///< SBUS串口对象指针
  SbusT sbus_msg_;              ///< SBUS消息对象
  
  // 通道参数
  int rx_min_;                  ///< 接收最小值
  int rx_max_;                  ///< 接收最大值
  int out_min_;                 ///< 输出最小值
  int out_max_;                 ///< 输出最大值
  int deadband_;                ///< 死区范围
  bool silent_failsafe_;        ///< 静默故障保护标志
  int channel_;                 ///< 通道数量
  double proportional_min_;     ///< 比例最小值
  double proportional_max_;     ///< 比例最大值
  float raw_span_;              ///< 原始值范围
  float out_span_;              ///< 输出值范围

  // 状态变量
  rclcpp::Time last_published_timestamp_; ///< 最后发布时间戳
  PoseT pose_;                    ///< 当前位姿
  VelT vel_;                      ///< 当前速度
  StateT state_ = NONE;           ///< 当前状态
  CmdT cmd_;                      ///< 当前命令

  // 常量定义
  static constexpr double SPEED_MAX = 2.0; ///< 最大线速度(m/s)
  static constexpr double ANGULAR_MAX = 1.0; ///< 最大角速度(rad/s)

  const tf2::Vector3 basket_vec_{0, 0, 0}; ///< 篮筐坐标(目标点)

  // ROS2接口
  rclcpp::Publisher<CmdT>::SharedPtr cmd_pub_;  ///< 命令发布器
  rclcpp::Publisher<VelT>::SharedPtr vel_pub_;  ///< 速度发布器
  rclcpp::Subscription<PoseStampT>::SharedPtr pose_sub_; ///< 位姿订阅器
  rclcpp::Publisher<SbusT>::SharedPtr sbus_pub_; ///< SBUS数据发布器
  rclcpp::TimerBase::SharedPtr timer_;          ///< 定时器

public:
  /**
   * @brief 构造函数
   * 
   * 初始化节点，声明参数，创建发布订阅，启动SBUS串口
   */
  sbus_node() : Node("sbus_node") {
    last_published_timestamp_ = this->get_clock()->now();
    // 声明所有参数
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

    // 获取参数并打印调试信息
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

    // 计算范围
    raw_span_ = static_cast<float>(rx_max_ - rx_min_);
    out_span_ = static_cast<float>(out_max_ - out_min_);
    
    // 创建发布器
    cmd_pub_ = this->create_publisher<CmdT>(cmd_topic_, 10);
    vel_pub_ = this->create_publisher<VelT>(vel_topic_, 10);
    sbus_pub_ = this->create_publisher<SbusT>(sbus_topic_, 10);

    // 创建位姿订阅器
    pose_sub_ = this->create_subscription<PoseStampT>(
        pose_topic_, 10,
        std::bind(&sbus_node::pose_feedback, this, std::placeholders::_1));

    // 初始化SBUS串口
    try {
      port_ = new sbus_serial::SBusSerialPort(port_name_, true);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unable to initalize SBUS port");
      throw;
    }

    sbus_msg_ = rc_interaction::msg::Sbus();

    // SBUS数据回调函数
    auto callback = [&](const sbus_serial::SBusMsg msg_) {
      constexpr auto MIDDLE = 1000;   // 摇杆中位值

      // 检查故障状态
      if (silent_failsafe_ && msg_.failsafe)
        return;
      if (msg_.frame_lost)
        return;
      
      // 计算线速度x分量(来自通道2)
      vel_.twist.linear.x = (msg_.channels[2] - MIDDLE) * 100.0f / 666;
      vel_.twist.linear.x = SPEED_MAX * vel_.twist.linear.x / 100;

      // 计算线速度y分量(来自通道3)
      vel_.twist.linear.y = (msg_.channels[3] - MIDDLE) * 100.0f / 666;
      vel_.twist.linear.y = SPEED_MAX * vel_.twist.linear.y / 100;

      // 计算角速度z分量(来自通道0)
      vel_.twist.angular.z = (msg_.channels[0] - MIDDLE) * 100.0f / 666;
      vel_.twist.angular.z = ANGULAR_MAX * vel_.twist.angular.z / 100;
      
      // 设置时间戳
      vel_.header.stamp = this->get_clock()->now();
      vel_.header.frame_id = "";
      
      // TODO: 处理按钮命令
      cmd_.data = state_;
      
      

      // TODO: 处理SBUS消息
    };

    port_->setCallback(callback);

    // 创建定时器
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000 / refresh_hz_),
                                 std::bind(&sbus_node::timer_callback, this));
    RCLCPP_INFO(this->get_logger(),
                "SBUS node started, publisher created, timer scheduled...");
  }

private:
  /**
   * @brief 位姿反馈回调函数
   * @param msg 位姿消息
   * 
   * 处理来自定位模块的位姿信息，计算机器人朝向目标的角度
   */
  void pose_feedback(const PoseStampT::ConstSharedPtr &msg) {
    pose_ = msg->pose;
    
    // 从四元数转换为欧拉角
    tf2::Quaternion pose_quat_(pose_.orientation.x, pose_.orientation.y,
                               pose_.orientation.z, pose_.orientation.w);
    tf2::Matrix3x3 m(pose_quat_);
    double r, p, y;
    m.getRPY(r, p, y);

    /* 坐标系说明:
     * 右上是零点,向左x递增向下y递增,y轴的负方向上yaw=0
     * yaw 逆时针为正,angle 顺时针为正
     * 
     * <---*----------
     *  | /\ |       |
     *  |/__\|       |
     *              |
     *              |
     *              |
     *              ↓
     */
    
    // 计算机器人到目标点的角度
    // 使用atan2计算从机器人到目标点的角度 (相对于y轴)
    // 公式: angle = atan2(dx, dy) 
    // 其中: dx = target_x - robot_x
    //       dy = target_y - robot_y
    // 注意: 坐标系y轴向下为正，因此角度方向需要特殊处理
    double angle =
        std::atan2((pose_.position.x - basket_vec_.x()), pose_.position.y);
    
    // 调试信息: 显示期望角度和当前角度
    ROBOT_INFO_F("desire yaw: %f ; current yaw: %f ; robot turn %s", angle, y,
                 angle > y ? "right" : "left");
    
    // 计算偏航角速度
    // 处理不同象限的角度差计算:
    // 1. 两者同为正角度
    // 2. 两者同为负角度 
    // 3. 当前角度正，目标角度负
    // 4. 当前角度负，目标角度正
    double delta_angle;
    if(angle>0 && y>0){
      // 两者都在右侧区域
      delta_angle = angle - y;
    }
    else if(angle<0 && y<0) {
      // 两者都在左侧区域
      delta_angle = angle - y;
    }
    else if(y>0){
      // 当前角度在右，目标角度在左
      // 需要处理跨越±π的情况
      delta_angle = (angle > 0) ? (angle - y) : ((M_PI + angle) + (M_PI - y));
    }
    else { 
      // 当前角度在左，目标角度在右
      // 需要处理跨越±π的情况
      delta_angle = (angle < 0) ? (angle - y) : ((-M_PI + angle) - (M_PI + y));
    }
    
    // 限制角速度范围
    delta_angle = std::clamp(delta_angle, -ANGULAR_MAX, ANGULAR_MAX);

    // TODO: 计算偏航角速度
    // vel_.twist.angular.x;
    return;
  }

  /**
   * @brief 定时器回调函数
   * 
   * 定期发布SBUS数据、速度命令和控制命令
   * 发布条件:
   * - 只有当速度命令时间戳更新时才发布
   * - 防止重复发布相同命令
   * 
   * 注意:
   * - 发布频率由refresh_hz参数控制
   * - 确保不会在短时间内发布过多消息
   */
  void timer_callback() {
    auto now = this->get_clock()->now();
    if ((now - last_published_timestamp_).seconds() > 0.1) {
      // 发布SBUS原始数据
      sbus_pub_->publish(sbus_msg_);
      
      // 发布速度命令
      vel_pub_->publish(vel_);
      
      // 发布控制命令
      cmd_pub_->publish(cmd_);
      
      // 更新时间戳
      last_published_timestamp_ = now;
      //打印数据
      cout<<"x:"<<vel_.twist.linear.x<<endl;
      cout<<"y:"<<vel_.twist.linear.y<<endl;
      // 调试信息
      RCLCPP_DEBUG(this->get_logger(), 
                  "Published commands - Vel: (%.2f,%.2f,%.2f), Cmd: %d",
                  vel_.twist.linear.x,
                  vel_.twist.linear.y,
                  vel_.twist.angular.z,
                  cmd_.data);
    }
  }
};
} // namespace robot
