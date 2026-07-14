#include "ros_bridge_singleton.h"
#include <argos3/core/utility/logging/argos_log.h>

RosBridgeSingleton::RosBridgeSingleton() {
    // Initialize ROS context once
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions options;
    options.auto_initialize_logging(false);
    context_->init(0, nullptr, options);

    // Create single node
    node_ = std::make_shared<rclcpp::Node>(
        "argos_ros_bridge", rclcpp::NodeOptions().context(context_));

    std::cerr << "[RosBridgeSingleton] ROS initialized\n";
}