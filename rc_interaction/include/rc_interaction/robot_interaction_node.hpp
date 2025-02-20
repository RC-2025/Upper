#pragma once

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <math.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <rc_interaction/basic_debugger.hpp>
#include <rc_interaction/utils.hpp>
namespace robot {
class interaction_node : public rclcpp::Node {

public:
  using JoyT = sensor_msgs::msg::Joy;
  using JoyFeedbackT = sensor_msgs::msg::JoyFeedbackArray;
  using PointT = geometry_msgs::msg::Point;
  using VelCovT = geometry_msgs::msg::TwistWithCovariance;
  using VelT = geometry_msgs::msg::Twist;
  using CmdT = std_msgs::msg::Int8;
  using PoseStampT = geometry_msgs::msg::PoseStamped;
  using PoseT = geometry_msgs::msg::Pose;

  enum Inst {
    L_HORIZON = 0, // left
    L_VERTICAL = 1,
    R_HORIZON = 3, // right
    R_VERTICAL = 4,
    C_HORIZON = 6,
    C_VERTICAL = 7, // cross button

    B_A = 0, // plain  button
    B_B = 1,
    B_X = 2,
    B_Y = 3,
    B_LL = 4,
    B_RR = 5,
    B_SEL = 6,
    B_START = 7,
    B_C = 9,
    B_Z = 10
  };
  // Inst 指代按键以及摇杆在数据里的下标位置

  enum State { MOV, SHOOT, NONE };

  void set_feedback(const JoyFeedbackT::ConstSharedPtr &msg) {
    if (ff_fd_ == -1) {
      return; // we arent ready yet
    }

    size_t size = msg->array.size();
    for (size_t i = 0; i < size; i++) {
      // process each feedback
      if (msg->array[i].type == 1 && ff_fd_ != -1) { // TYPE_RUMBLE
        // if id is zero, thats low freq, 1 is high
        joy_effect_.direction = 0; // down
        joy_effect_.type = FF_RUMBLE;
        if (msg->array[i].id == 0) {
          joy_effect_.u.rumble.strong_magnitude =
              (static_cast<float>(1 << 15)) * msg->array[i].intensity;
        } else {
          joy_effect_.u.rumble.weak_magnitude =
              (static_cast<float>(1 << 15)) * msg->array[i].intensity;
        }

        joy_effect_.replay.length = 1000;
        joy_effect_.replay.delay = 0;

        update_feedback_ = true;
      }
    }
  }

  void pose_feedback(const PoseStampT::ConstSharedPtr &msg) {

    pose_ = msg->pose;
    tf2::Quaternion pose_quat_(pose_.orientation.x, pose_.orientation.y,
                               pose_.orientation.z, pose_.orientation.w);
    tf2::Matrix3x3 m(pose_quat_);
    double r, p, y;
    m.getRPY(r, p, y);

    //右上是零点,向左x递增向下y递增,y轴的负方向上yaw=0
    /**
    <--------------
                  |
                  |
                  |
                  |
                  |
                  ↓
     */
    double angle =
        std::atan2((pose_.position.x - basket_vec_.x()), pose_.position.y);
    ROBOT_INFO_F("desire yaw: %f ; current yaw: %f ; robot turn %s", angle, y,
                 angle > y ? "right" : "left");
    // TODO
    // calculate yaw speed
    // vel_.angular.x;
    return;
  }

  interaction_node() : Node("interaction_node"), ff_fd_(-1) {

// diagnostic_updater
#ifdef _DIAGNOSTIC
    diagnostic_ = std::make_shared<diagnostic_updater::Updater>(this);
    diagnostic_->add("interaction_node Driver Status", this,
                     &interaction_node::diagnostics);
    diagnostic_->setHardwareID("none");
#endif

    // pub
    joy_pub_ = this->create_publisher<JoyT>("joy", 10);
    cmd_pub_ = this->create_publisher<CmdT>("cmd_behav", 10);
    vel_pub_ = this->create_publisher<VelT>("cmd_vel", 10);

    // sub
    rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr sub_ =
        this->create_subscription<sensor_msgs::msg::JoyFeedbackArray>(
            "joy/set_feedback", rclcpp::QoS(10),
            std::bind(&interaction_node::set_feedback, this,
                      std::placeholders::_1));
    pose_sub_ = this->create_subscription<PoseStampT>(
        "/mcl_pose", 10,
        std::bind(&interaction_node::pose_feedback, this,
                  std::placeholders::_1));

    joy_dev_ = this->declare_parameter("dev", std::string("/dev/input/js0"));
    joy_dev_name_ = this->declare_parameter("dev_name", std::string(""));
    joy_dev_ff_ = this->declare_parameter("dev_ff", "/dev/input/event0");
    deadzone_ = this->declare_parameter("deadzone", 0.05);
    autorepeat_rate_ = this->declare_parameter("autorepeat_rate", 20.0);
    coalesce_interval_ = this->declare_parameter("coalesce_interval", 0.001);
    default_trig_val_ = this->declare_parameter("default_trig_val", false);
    sticky_buttons_ = this->declare_parameter("sticky_buttons", false);

    GET_PARAM_DEBUG("dev", joy_dev_);
    GET_PARAM_DEBUG("dev_name", joy_dev_name_);
    GET_PARAM_DEBUG("dev_ff", joy_dev_ff_);
    GET_PARAM_DEBUG("deadzone", deadzone_);
    GET_PARAM_DEBUG("autorepeat_rate", autorepeat_rate_);
    GET_PARAM_DEBUG("coalesce_interval", coalesce_interval_);
    GET_PARAM_DEBUG("default_trig_val", default_trig_val_);
    GET_PARAM_DEBUG("sticky_buttons", sticky_buttons_);

    // Checks on parameters
    if (!joy_dev_name_.empty()) {
      std::string joy_dev_path =
          get_dev_by_joy_name(joy_dev_name_, this->get_logger());
      if (joy_dev_path.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Couldn't find a joystick with name %s. "
                     "Falling back to default device.",
                     joy_dev_name_.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Using %s as joystick device.",
                    joy_dev_path.c_str());
        joy_dev_ = joy_dev_path;
      }
    }

    if (autorepeat_rate_ > 1 / coalesce_interval_) {
      RCLCPP_WARN(this->get_logger(),
                  "interaction_node: autorepeat_rate (%f Hz) > "
                  "1/coalesce_interval (%f Hz) does not make sense. Timing "
                  "behavior is not well defined.",
                  autorepeat_rate_, 1 / coalesce_interval_);
    }

    if (deadzone_ >= 1) {
      RCLCPP_WARN(this->get_logger(),
                  "interaction_node: deadzone greater than 1 was requested. "
                  "The semantics of deadzone have changed. It is now related "
                  "to the range [-1:1] instead "
                  "of [-32767:32767]. For now I am dividing your deadzone by "
                  "32767, but this behavior is "
                  "deprecated so you need to update your launch file.");
      deadzone_ /= 32767;
    }

    if (deadzone_ > 0.9) {
      RCLCPP_WARN(
          this->get_logger(),
          "interaction_node: deadzone (%f) greater than 0.9, setting it to 0.9",
          deadzone_);
      deadzone_ = 0.9;
    }

    if (deadzone_ < 0) {
      RCLCPP_WARN(this->get_logger(),
                  "interaction_node: deadzone_ (%f) less than 0, setting to 0.",
                  deadzone_);
      deadzone_ = 0;
    }

    if (autorepeat_rate_ < 0) {
      RCLCPP_WARN(
          this->get_logger(),
          "interaction_node: autorepeat_rate (%f) less than 0, setting to 0.",
          autorepeat_rate_);
      autorepeat_rate_ = 0;
    }

    if (coalesce_interval_ < 0) {
      RCLCPP_WARN(
          this->get_logger(),
          "interaction_node: coalesce_interval (%f) less than 0, setting to 0.",
          coalesce_interval_);
      coalesce_interval_ = 0;
    }

    // Parameter conversions
    double autorepeat_interval = 1 / autorepeat_rate_;
    double scale = -1. / (1. - deadzone_) / 32767.;
    double unscaled_deadzone = 32767. * deadzone_;

    // TODO
    //手柄scale到实际速度的转换

    js_event event;
    struct timeval tv;
    fd_set set;
    int joy_fd;

    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = this->now().seconds();

    // Big while loop opens, publishes
    while (rclcpp::ok()) {
      open_ = false;

#ifdef _DIAGNOSTIC
      diagnostic_->force_update();
#endif

      bool first_fault = true;
      while (true) {
        // In the first iteration of this loop, first_fault is true so we just
        // want to check for rclcpp work and not block.  If it turns out that
        // we cannot open the joystick device immediately, then in subsequent
        // iterations we block for up to a second in rclcpp before attempting
        // to open the joystick device again.  The dummy promise and future
        // are used to accomplish this 1 second wait.
        std::promise<void> dummy_promise;
        std::shared_future<void> dummy_future(dummy_promise.get_future());
        std::chrono::duration<int64_t, std::milli> timeout;
        if (first_fault) {
          timeout = std::chrono::milliseconds(0);
        } else {
          timeout = std::chrono::milliseconds(1000);
        }
        rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           dummy_future, timeout);
        if (!rclcpp::ok()) {
          goto cleanup;
        }
        joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        if (joy_fd != -1) {
          // There seems to be a bug in the driver or something where the
          // initial events that are to define the initial state of the
          // joystick are not the values of the joystick when it was opened
          // but rather the values of the joystick when it was last closed.
          // Opening then closing and opening again is a hack to get more
          // accurate initial state data.
          close(joy_fd);
          joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        }
        if (joy_fd != -1) {
          break;
        }
        if (first_fault) {
          RCLCPP_ERROR(this->get_logger(),
                       "Couldn't open joystick %s. Will retry every second.",
                       joy_dev_.c_str());
          first_fault = false;
        }
      }

      if (!joy_dev_ff_.empty()) {
        ff_fd_ = open(joy_dev_ff_.c_str(), O_RDWR);

        /* Set the gain of the device*/
        int gain = 100; /* between 0 and 100 */
        struct input_event
            ie; /* structure used to communicate with the driver */

        ie.type = EV_FF;
        ie.code = FF_GAIN;
        ie.value = 0xFFFFUL * gain / 100;

        if (write(ff_fd_, &ie, sizeof(ie)) == -1) {
          RCLCPP_WARN(this->get_logger(),
                      "Couldn't open joystick force feedback: %s",
                      strerror(errno));
        }

        joy_effect_.id = -1;
        joy_effect_.direction = 0; // down
        joy_effect_.type = FF_RUMBLE;
        joy_effect_.u.rumble.strong_magnitude = 0;
        joy_effect_.u.rumble.weak_magnitude = 0;
        joy_effect_.replay.length = 1000;
        joy_effect_.replay.delay = 0;

        // upload the effect
        // FIXME: check the return value here
        ioctl(ff_fd_, EVIOCSFF, &joy_effect_);
      }

      RCLCPP_INFO(this->get_logger(), "Opened joystick: %s. deadzone_: %f.",
                  joy_dev_.c_str(), deadzone_);
      open_ = true;

#ifdef _DIAGNOSTIC
      diagnostic_->force_update();
#endif

      bool tv_set = false;
      bool publication_pending = false;
      tv.tv_sec = 1;
      tv.tv_usec = 0;

      // Here because we want to reset it on device close.
      auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
      double val; // Temporary variable to hold event values
      joy_msg->header.frame_id = "joy";

      while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());

        bool publish_now = false;
        bool publish_soon = false;
        FD_ZERO(&set);
        FD_SET(joy_fd, &set);

        int select_out = select(joy_fd + 1, &set, nullptr, nullptr, &tv);
        if (select_out == -1) {
          tv.tv_sec = 0;
          tv.tv_usec = 0;
          continue;
        }

        // play the rumble effect (can probably do this at lower rate later)
        if (ff_fd_ != -1) {
          struct input_event start;
          start.type = EV_FF;
          start.code = joy_effect_.id;
          start.value = 3;
          if (write(ff_fd_, (const void *)&start, sizeof(start)) == -1) {
            break; // fd closed
          }

          // upload the effect
          if (update_feedback_ == true) {
            // FIXME: check the return value here.
            ioctl(ff_fd_, EVIOCSFF, &joy_effect_);
            update_feedback_ = false;
          }
        }

        if (FD_ISSET(joy_fd, &set)) {
          if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN) {
            break; // interaction_node is probably closed. Definitely occurs.
          }

          joy_msg->header.stamp = this->now();
          event_count_++;
          switch (event.type) {
          case JS_EVENT_BUTTON:
          case JS_EVENT_BUTTON | JS_EVENT_INIT:
            if (event.number >= joy_msg->buttons.size()) {
              size_t old_size = joy_msg->buttons.size();
              joy_msg->buttons.resize(event.number + 1);
              for (size_t i = old_size; i < joy_msg->buttons.size(); i++) {
                joy_msg->buttons[i] = 0.0;
              }
            }
            if (sticky_buttons_) {
              if (event.value == 1) {
                joy_msg->buttons[event.number] =
                    1 - joy_msg->buttons[event.number];
              }
            } else {
              joy_msg->buttons[event.number] = (event.value ? 1 : 0);
            }
            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            if (!(event.type & JS_EVENT_INIT)) {
              publish_now = true;
            } else {
              publish_soon = true;
            }
            break;
          case JS_EVENT_AXIS:
          case JS_EVENT_AXIS | JS_EVENT_INIT:
            val = event.value;
            if (event.number >= joy_msg->axes.size()) {
              size_t old_size = joy_msg->axes.size();
              joy_msg->axes.resize(event.number + 1);
              for (size_t i = old_size; i < joy_msg->axes.size(); i++) {
                joy_msg->axes[i] = 0.0;
              }
            }
            if (default_trig_val_) {
              // Allows deadzone to be "smooth"
              if (val > unscaled_deadzone) {
                val -= unscaled_deadzone;
              } else if (val < -unscaled_deadzone) {
                val += unscaled_deadzone;
              } else {
                val = 0;
              }
              joy_msg->axes[event.number] = val * scale;
              // Will wait a bit before sending to try to combine events.
              publish_soon = true;
              break;
            } else {
              if (!(event.type & JS_EVENT_INIT)) {
                val = event.value;
                if (val > unscaled_deadzone) {
                  val -= unscaled_deadzone;
                } else if (val < -unscaled_deadzone) {
                  val += unscaled_deadzone;
                } else {
                  val = 0;
                }
                joy_msg->axes[event.number] = val * scale;
              }

              publish_soon = true;
              break;
            }
          default:
            RCLCPP_WARN(
                this->get_logger(),
                "interaction_node: Unknown event type. "
                "Please file a ticket. time=%u, value=%d, type=%Xh, number=%d",
                event.time, event.value, event.type, event.number);
            break;
          }
        } else if (tv_set) { // Assume that the timer has expired.
          joy_msg->header.stamp = this->now();
          publish_now = true;
        }

        if (publish_now) {
          // Assume that all the JS_EVENT_INIT messages have arrived already.
          // This should be the case as the kernel sends them along as soon as
          // the device opens.

          VelT vel_msg_;
          auto button_press_index = static_cast<size_t>(
              std::find(joy_msg->buttons.begin(), joy_msg->buttons.end(), 1) -
              joy_msg->buttons.begin());
          switch (button_press_index) {
          case B_A:
            ROBOT_INFO_F("press %s", "B_A");
            break;
          case B_B:
            ROBOT_INFO_F("press %s", "B_B");
            break;
          case B_X:
            ROBOT_INFO_F("press %s", "B_X");
            break;
          case B_Y:
            ROBOT_INFO_F("press %s", "B_Y");
            break;
          case B_LL:
            ROBOT_INFO_F("press %s", "B_LL");
            break;
          case B_RR:
            ROBOT_INFO_F("press %s", "B_RR");
            break;
          case B_SEL:
            ROBOT_INFO_F("press %s", "B_SEL");
            break;
          case B_START:
            ROBOT_INFO_F("press %s", "B_START");
            break;
          case B_C:
            ROBOT_INFO_F("press %s", "B_C");
            break;
          case B_Z:
            ROBOT_INFO_F("press %s", "B_Z");
            break;
          default:
            // ROBOT_INFO("none press");
            break;
          }
          // TODO
          // pub cmd_vel and cmd_behv
          // TODO
          if (state_ == SHOOT) {
            vel_msg_.angular = vel_.angular;
          }
          // TODO
          cmd_.data = NONE;

          joy_msg->header.stamp = this->now();
          joy_pub_->publish(*joy_msg);
          cmd_pub_->publish(cmd_);
          vel_pub_->publish(vel_msg_);

          publish_now = false;
          tv_set = false;
          publication_pending = false;
          publish_soon = false;
          pub_count_++;
        }

        // If an axis event occurred, start a timer to combine with other
        // events.
        if (!publication_pending && publish_soon) {
          tv.tv_sec = trunc(coalesce_interval_);
          tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
          publication_pending = true;
          tv_set = true;
        }

        // If nothing is going on, start a timer to do autorepeat.
        if (!tv_set && autorepeat_rate_ > 0) {
          tv.tv_sec = trunc(autorepeat_interval);
          tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
          tv_set = true;
        }

        if (!tv_set) {
          tv.tv_sec = 1;
          tv.tv_usec = 0;
        }
      } // End of joystick open loop.

      close(ff_fd_);
      close(joy_fd);
      rclcpp::spin_some(this->get_node_base_interface());
      if (rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Connection to joystick device lost unexpectedly. Will reopen.");
      }
    }

  cleanup:
    RCLCPP_INFO(this->get_logger(), "interaction_node shut down.");
  }

private:
  bool open_;
  bool sticky_buttons_;
  bool default_trig_val_;
  std::string joy_dev_;
  std::string joy_dev_name_;
  std::string joy_dev_ff_;
  double deadzone_;
  double autorepeat_rate_;   // in Hz.  0 for no repeat.
  double coalesce_interval_; // Defaults to 100 Hz rate limit.
  int event_count_;
  int pub_count_;
  double lastDiagTime_;
  int ff_fd_;
  struct ff_effect joy_effect_;
  bool update_feedback_;
  PoseT pose_;
  VelT vel_;
  State state_ = NONE;
  CmdT cmd_;

  const tf2::Vector3 basket_vec_{3, 0, 0};

  rclcpp::Publisher<JoyT>::SharedPtr joy_pub_;
  rclcpp::Publisher<CmdT>::SharedPtr cmd_pub_;
  rclcpp::Publisher<VelT>::SharedPtr vel_pub_;

  rclcpp::Subscription<PoseStampT>::SharedPtr pose_sub_;

// /\brief Publishes diagnostics and status
#ifdef _DIAGNOSTIC

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_;

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    double now = this->now().seconds();
    double interval = now - lastDiagTime_;
    if (open_) {
      stat.summary(0, "OK");
    } else {
      stat.summary(2, "interaction_node not open.");
    }

    stat.add("topic", joy_pub_->get_topic_name());
    stat.add("device", joy_dev_);
    stat.add("device name", joy_dev_name_);
    stat.add("dead zone", deadzone_);
    stat.add("autorepeat rate (Hz)", autorepeat_rate_);
    stat.add("coalesce interval (s)", coalesce_interval_);
    stat.add("recent joystick event rate (Hz)", event_count_ / interval);
    stat.add("recent publication rate (Hz)", pub_count_ / interval);
    stat.add("subscribers", joy_pub_->get_subscription_count());
    stat.add("default trig val", default_trig_val_);
    stat.add("sticky buttons", sticky_buttons_);
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = now;
  }
#endif

  /*! \brief Returns the device path of the first joystick that matches
   * joy_name. If no match is found, an empty string is returned.
   */
  std::string get_dev_by_joy_name(const std::string &joy_name,
                                  rclcpp::Logger logger) {
    const char path[] = "/dev/input"; // no trailing / here
    struct dirent *entry;
    struct stat stat_buf;

    DIR *dev_dir = opendir(path);
    if (dev_dir == nullptr) {
      RCLCPP_ERROR(logger, "Couldn't open %s. Error %i: %s.", path, errno,
                   strerror(errno));
      return "";
    }

    while ((entry = readdir(dev_dir)) != nullptr) {
      // filter entries
      if (strncmp(entry->d_name, "js", 2) !=
          0) { // skip device if it's not a joystick
        continue;
      }
      std::string current_path = std::string(path) + "/" + entry->d_name;
      if (stat(current_path.c_str(), &stat_buf) == -1) {
        continue;
      }
      if (!S_ISCHR(stat_buf.st_mode)) { // input devices are character devices,
                                        // skip other
        continue;
      }

      // get joystick name
      int joy_fd = open(current_path.c_str(), O_RDONLY);
      if (joy_fd == -1) {
        continue;
      }

      char current_joy_name[128];
      if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)),
                current_joy_name) < 0) {
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));
      }

      close(joy_fd);

      RCLCPP_INFO(logger, "Found joystick: %s (%s).", current_joy_name,
                  current_path.c_str());

      if (strcmp(current_joy_name, joy_name.c_str()) == 0) {
        closedir(dev_dir);
        return current_path;
      }
    }

    closedir(dev_dir);
    return "";
  }
};
} // namespace robot