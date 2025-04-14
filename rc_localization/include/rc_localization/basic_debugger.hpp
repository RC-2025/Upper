#pragma once

#define ANSI_FG_BLACK "\33[1;30m"
#define ANSI_FG_RED "\33[1;31m"
#define ANSI_FG_GREEN "\33[1;32m"
#define ANSI_FG_YELLOW "\33[1;33m"
#define ANSI_FG_BLUE "\33[1;34m"
#define ANSI_FG_MAGENTA "\33[1;35m"
#define ANSI_FG_CYAN "\33[1;36m"
#define ANSI_FG_WHITE "\33[1;37m"
#define ANSI_BG_BLACK "\33[1;40m"
#define ANSI_BG_RED "\33[1;41m"
#define ANSI_BG_GREEN "\33[1;42m"
#define ANSI_BG_YELLOW "\33[1;43m"
#define ANSI_BG_BLUE "\33[1;44m"
#define ANSI_BG_MAGENTA "\33[1;35m"
#define ANSI_BG_CYAN "\33[1;46m"
#define ANSI_BG_WHITE "\33[1;47m"
#define ANSI_NONE "\33[0m"

#define ROBOT_INFO_F(fmt, ...)                                                 \
  RCLCPP_INFO(this->get_logger(), ANSI_FG_BLUE fmt ANSI_NONE, __VA_ARGS__)

#define ROBOT_WARN_F(fmt, ...)                                                 \
  RCLCPP_WARN(this->get_logger(), ANSI_BG_YELLOW fmt ANSI_NONE, __VA_ARGS__)

#define ROBOT_ERROR_F(fmt, ...)                                                \
  RCLCPP_WARN(this->get_logger(), ANSI_BG_RED fmt ANSI_NONE, __VA_ARGS__)

#define ROBOT_INFO(...)                                                        \
  RCLCPP_INFO(this->get_logger(), ANSI_FG_BLUE __VA_ARGS__ ANSI_NONE)

#define ROBOT_WARN(...)                                                        \
  RCLCPP_WARN(this->get_logger(), ANSI_BG_YELLOW __VA_ARGS__ ANSI_NONE)

#define ROBOT_ERROR(...)                                                       \
  RCLCPP_WARN(this->get_logger(), ANSI_BG_RED __VA_ARGS__ ANSI_NONE)

#define ROBOT_INFO_STRAM(stream)                                               \
  RCLCPP_INFO_STREAM(this->get_logger(), ANSI_FG_BLUE << stream << ANSI_NONE)

#define ROBOT_WARN_STRAM(stream)                                               \
  RCLCPP_WARN_STREAM(this->get_logger(), ANSI_BG_YELLOW << stream << ANSI_NONE)