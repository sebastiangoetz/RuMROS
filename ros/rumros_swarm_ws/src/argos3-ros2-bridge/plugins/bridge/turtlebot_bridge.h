/*
 * turtlebot_bridge.h (based on footbot_bridge.h)
 * Author: Alexander Kassuba
 */

#ifndef TURTLEBOT_BRIDGE_H_
#define TURTLEBOT_BRIDGE_H_

#include "base/robot_bridge_base.h"

// Sensors and actuators
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h> // Not yet implemented in turtlebot plugin
#include <argos3/plugins/robots/turtlebot3/control_interface/ci_turtlebot3_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h> // Not yet implemented in turtlebot plugin
#include <argos3/plugins/robots/turtlebot3/control_interface/ci_turtlebot3_lidar_sensor.h>

/**
 * ROS2 Imports
 */
#include "rclcpp/rclcpp.hpp"
#include "rumros_msgs/msg/light_list.hpp"
#include "rumros_msgs/msg/proximity_list.hpp"
#include "rumros_msgs/msg/led.hpp"
#include "rumros_msgs/msg/lidar_scan.hpp"


using namespace argos;

/**
 * @brief Implements the ROS bridge for a simulated Turtlebot3.
 * 
 */
class TurtlebotBridge : public RobotBridgeBase {
    private:

		// Publishers
		rclcpp::Publisher<rumros_msgs::msg::LightList>::SharedPtr groundListPublisher_;
		rclcpp::Publisher<rumros_msgs::msg::ProximityList>::SharedPtr promixityListPublisher_;
		rclcpp::Publisher<rumros_msgs::msg::LightList>::SharedPtr lightListPublisher_;
		// Not yet implemented (turtlebot plugin): ultrasound sensor publisher
		rclcpp::Publisher<rumros_msgs::msg::LidarScan>::SharedPtr lidarScanPublisher_;

		// Subscribers
		rclcpp::Subscription<rumros_msgs::msg::Led>::SharedPtr cmdLedSubscriber_;

		// Actuators
		CCI_LEDsActuator* m_pcLEDs{nullptr};

		// Sensors
		CCI_GroundSensor* m_pcGround{nullptr};
		CCI_Turtlebot3ProximitySensor* m_pcProximity{nullptr};
		CCI_LightSensor* m_pcLight{nullptr};
		//CCI_Turtlebot3UltrasoundSensor* m_pcUltrasound{nullptr}; // Not yet implemented
		CCI_Turtlebot3LIDARSensor* m_pcLidar{nullptr};

    public:

		TurtlebotBridge();
		~TurtlebotBridge() override;

		void Init(TConfigurationNode& t_node) override;

		void ControlStepHook() override;

		// The callback method for getting new commanded led color on the cmd_led topic
		void cmdLedCallback(const rumros_msgs::msg::Led& ledColor);
};
#endif /* TURTLEBOT_BRIDGE_H_ */