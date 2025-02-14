#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <rc_localization/basic_debugger.hpp>
#include <rc_localization/utils.hpp>
namespace robot {
class localization_node : public rclcpp::Node {

public:
  using MapT = nav_msgs::msg::OccupancyGrid;

  localization_node() : Node("localization_node") {

    this->declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
    this->declare_parameter("map_topic_name", "map");
    this->declare_parameter("frame_id", "map");

    GET_PARAM_DEBUG("yaml_filename", yaml_filename);
    GET_PARAM_DEBUG("map_topic_name", map_topic_name_);
    GET_PARAM_DEBUG("frame_id", map_frame_id_);

    map_pub_ = this->create_publisher<MapT>(
        map_topic_name_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    if (!yaml_filename.empty()) {
      if (nav2_map_server::LOAD_MAP_SUCCESS !=
          nav2_map_server::loadMapFromYaml(yaml_filename, map_)) {
        ROBOT_ERROR_F("Error when load map from %s", yaml_filename.c_str());
        exit(1);
      }
    } else {
      ROBOT_ERROR("map filename is empty.");
    }

    map_.header.frame_id = map_frame_id_;
    map_.header.stamp = this->now();
    map_.info.map_load_time = this->now();
    map_pub_->publish(map_);
  }

private:
  rclcpp::Publisher<MapT>::SharedPtr map_pub_;

  MapT map_;

  std::string yaml_filename;
  std::string map_topic_name_;
  std::string map_frame_id_;
};
} // namespace robot