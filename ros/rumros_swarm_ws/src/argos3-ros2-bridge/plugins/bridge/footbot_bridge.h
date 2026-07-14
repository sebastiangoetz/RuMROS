/*
 * footbot_bridge.h (modified to work with RuMROS)
 *
 *  Created on: 28 Jan 2025
 *  Original author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

#ifndef FOOTBOT_BRIDGE_H_
#define FOOTBOT_BRIDGE_H_

#include "base/robot_bridge_base.h"
#include <chrono>

/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the distance scanner sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_sensor.h>
/* Definition of the perspective camera camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>


/**
 * ROS2 Imports
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rumros_msgs/msg/led.hpp"
#include "rumros_msgs/msg/packet.hpp"
#include "rumros_msgs/msg/position.hpp"
#include "rumros_msgs/msg/blob_list.hpp"
#include "rumros_msgs/msg/light_list.hpp"
#include "rumros_msgs/msg/packet_list.hpp"
#include "rumros_msgs/msg/proximity_list.hpp"

using namespace argos;
using namespace std::chrono_literals;

/**
 * @brief Implements the ROS bridge for a simulated Foot-Bot.
 * 
 */
class FootbotBridge : public RobotBridgeBase {
    private:
		// Publishers
		rclcpp::Publisher<rumros_msgs::msg::LightList>::SharedPtr lightListPublisher_;
		rclcpp::Publisher<rumros_msgs::msg::ProximityList>::SharedPtr promixityListPublisher_;
		rclcpp::Publisher<rumros_msgs::msg::BlobList>::SharedPtr blobListPublisher_;
		rclcpp::Publisher<rumros_msgs::msg::PacketList>::SharedPtr rabPublisher_;

		// Subscribers
		rclcpp::Subscription<rumros_msgs::msg::Packet>::SharedPtr cmdRabSubscriber_;
		rclcpp::Subscription<rumros_msgs::msg::Led>::SharedPtr cmdLedSubscriber_;

		// Actuators
		CCI_LEDsActuator* m_pcLEDs;
		CCI_RangeAndBearingActuator* m_pcRABA;

		// Sensors
		CCI_FootBotLightSensor* m_pcLight;
		CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
		CCI_FootBotProximitySensor* m_pcProximity;
		CCI_PositioningSensor* m_pcPosition;
		CCI_RangeAndBearingSensor* m_pcRABS;

    public:

		FootbotBridge();
		~FootbotBridge() override;

		void Init(TConfigurationNode& t_node) override;

		void ControlStepHook() override;
		
		//The callback method for getting new commanded packet on the cmd_packet topic
		void cmdRabCallback(const rumros_msgs::msg::Packet& packet);
		
		// The callback method for getting new commanded led color on the cmd_led topic
		void cmdLedCallback(const rumros_msgs::msg::Led& ledColor);
};
#endif /* FOOTBOT_BRIDGE_H_ */