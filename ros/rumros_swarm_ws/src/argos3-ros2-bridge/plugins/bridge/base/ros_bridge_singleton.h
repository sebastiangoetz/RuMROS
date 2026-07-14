#ifndef ROS_BRIDGE_SINGLETON_H_
#define ROS_BRIDGE_SINGLETON_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * @brief Isolates the communication structures with ROS, i.e. Node and
 * context in a singleton, such that they stay unique across all created
 * robot controllers.
 * 
 */
class RosBridgeSingleton {
public:
    static RosBridgeSingleton& Instance() {
        static RosBridgeSingleton instance;
        return instance;
    }

    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<rclcpp::Node> node_;

private:
    RosBridgeSingleton();
    
    // Prevent copies
    RosBridgeSingleton(const RosBridgeSingleton&) = delete;
    RosBridgeSingleton& operator=(const RosBridgeSingleton&) = delete;
};

#endif // ROS_BRIDGE_SINGLETON_H_