/*
 * crazyflie_bridge.h (TU Dresden)
 * Topics and Services mirror the implementation of Crazyswarm2's simulation server,
 * which is used to control Gazebo-simulated Crazyflies with ROS2. For more details,
 * see https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie_sim/crazyflie_sim/crazyflie_server.py.
 * Broadcasting to multiple drones is not currently supported, and will be
 * added in the future, if required.
 * 
 * An implementation of the ROS topics and services Crazyswarm2 uses for real
 * Crazyflies can be found at
 * https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie/src/crazyflie_server.cpp.
 * (Though they don't implement all topics/services in simulation)
 * 
 * The simulated CF drones are provided by the CF ARGoS plugin:
 * https://gitlab.com/uniluxembourg/snt/pcog/adars/crazyflie
 * and can be actuated via a simplified positional or velocity controller.
 * Software-in-the-loop (SIL) simulation was planned, but as of now, ARGoS cannot run
 * its control loop at a rate fast enough to sustain behavior in a working manner when
 * integrated with ROS. Therefore, this mode is not used for now.
 *
 *  Created on: 20 May 2026
 */

#ifndef CRAZYFLIE_BRIDGE_H
#define CRAZYFLIE_BRIDGE_H

#include "base/robot_bridge_base.h"
#include <chrono>

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include "cf_firmware.h"
#include "crazyflie_physics.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rumros_msgs/msg/blob_list.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"
#include "crazyflie_interfaces/msg/status.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace argos;
using namespace std::chrono_literals;
using namespace crazyflie_interfaces::srv;
using namespace std_srvs::srv;
using namespace crazyflie_interfaces::msg;
using namespace rumros_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace geometry_msgs::msg;

/**
 * @brief Implements the ROS bridge for a simulated Crazyflie 2.x drone.
 * Some native (non-ROS2) controller implementations can be found at
 * https://gitlab.com/uniluxembourg/snt/pcog/adars/crazyflie/-/tree/main/examples.
 * Here, we actuate the Crazyflie directly using the 
 * positional and velocity controllers. A SIL implementation with the firmware
 * was tested (see cf_firmware.h), but did not work due to the control
 loop running way too slowly (10 Hz vs 500+ Hz on CF).
 * 
 * Connection with ROS2 is realized via the Crazyswarm2 package
 * https://github.com/IMRCLab/crazyswarm2. This controller conforms
 * to the protocol of Crazyswarm2, implementing the same topics and
 * service handlers, which actuate the simulated drones.
 */
class CrazyflieBridge: public CCI_Controller
{
    public:

		// For non-SIL direct mode
		enum Mode
		{
			MODE_IDLE,
			MODE_HOVER,
			MODE_TAKEOFF,
			MODE_LAND,
			MODE_SETPOINT_STREAM,
			MODE_TRAJECTORY,
			MODE_FAILSAFE
		} mode;

		struct Setpoint
		{
			CVector3 position;
			float yaw = 0.0f;
			Real duration = 0.0; // Time to get there
		};

		CrazyflieBridge();
		~CrazyflieBridge();

		void Init(TConfigurationNode& t_node);

		virtual void ControlStep() override;

		// Interface functions
		void setLedColor(CColor color);
		inline CVector3 getQuadPosition();
		inline CQuaternion getQuadOrientation();

		inline Real getTime();

    private:
		std::string name;
		rclcpp::Node::SharedPtr nodeHandle;

		CVector3 currentPos = CVector3::ZERO;
		CQuaternion currentOrientation;

		Real descentRate = 0.5;
		Real prevTime = 0.0;
		int landInitTime = -1;
		Real timeStep;
		CSimulator* cSimulator;

		CFFirmware* firmware;
		CrazyfliePhysics* physics;

		struct takeoffLandData
		{
			CVector3 initialPosition;
			CVector3 goalPosition;
			float height; // Negative when landing
			float duration;
		} takeoffLandData;

		CVector3* prevPos = nullptr; // Position two seconds ago

		// For non-SIL direct mode
		std::vector<Setpoint> trajSetpoints;
		int iTrajSetpoint = -1;
		bool movingToSetpoint = false; // Set to true in modes where drone moves to a position
		Real tPrevSetpoint = -1.0;
		Real tSetpointTimeout = 1.0; // Timeout in MODE_SETPOINT_STREAM, after which MODE_FAILSAFE is triggered
		Setpoint* setpoint = nullptr;

		void moveToSetpoint(CVector3 goal, CRadians yaw=CRadians(0.0), bool relative=false);

		// === ROS TOPICS ===
		// Subscribers
		rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subCmdFullState;
		rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr subCmdPosition;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel;
		rclcpp::Subscription<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr subCmdVelWorld;
		rclcpp::Subscription<crazyflie_interfaces::msg::Hover>::SharedPtr subCmdHover;
		// TODO: I don't have the LED deck, and according to GitHub issues it
		// doesn't seem to be supported yet. For now, a simple topic is used
		// to switch the LED color, but this will likely be mismatched in a
		// later implementation and require adjustments.
		rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr subCmdLed;

		// Publishers
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubPose;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubRobotDescription;
		rclcpp::Publisher<crazyflie_interfaces::msg::Status>::SharedPtr pubStatus;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaserScan;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom;

		// === ROS SERVICES ===
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvEmergency;
		rclcpp::Service<crazyflie_interfaces::srv::StartTrajectory>::SharedPtr srvStartTrajectory;
		rclcpp::Service<crazyflie_interfaces::srv::Takeoff>::SharedPtr srvTakeoff;
		rclcpp::Service<crazyflie_interfaces::srv::Land>::SharedPtr srvLand;
		rclcpp::Service<crazyflie_interfaces::srv::GoTo>::SharedPtr srvGoTo;
		rclcpp::Service<crazyflie_interfaces::srv::UploadTrajectory>::SharedPtr srvUploadTrajectory;
		rclcpp::Service<crazyflie_interfaces::srv::NotifySetpointsStop>::SharedPtr srvSetpointsStop;
		rclcpp::Service<crazyflie_interfaces::srv::Arm>::SharedPtr srvArm;

		// === SENSORS & ACTUATORS ===
		// Actuators
        CCI_QuadRotorSpeedActuator* m_pcSpdAct;
        CCI_QuadRotorPositionActuator* m_pcPosAct;
        CCI_LEDsActuator* m_pcLedAct;

		// Sensors
        CCI_PositioningSensor* m_pcPosSens;

		// Cache sensor attachment booleans
		bool hasSpeedActuator = false;
		bool hasPosActuator = false;
		bool hasLedsActuator = false;

		bool hasPositioningSensor = false;
		bool hasCameraSensor = false;

		// Callbacks & Service handlers
		void cbCmdVelLegacy(geometry_msgs::msg::Twist::SharedPtr msg);
		void cbCmdVelWorld(crazyflie_interfaces::msg::VelocityWorld::SharedPtr msg);
		void cbCmdPosition(crazyflie_interfaces::msg::Position::SharedPtr msg);
		void cbCmdHover(crazyflie_interfaces::msg::Hover::SharedPtr msg);
		void cbCmdFullState(crazyflie_interfaces::msg::FullState::SharedPtr msg);
		void cbCmdLed(std_msgs::msg::ColorRGBA::SharedPtr msg);

		/**
		 * @brief Performs an emergency stop, halting all motors.
		 */
		std::shared_ptr<Empty_Response> sEmergency(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response);

		/**
		 * @brief Instructs the drone to start moving along a specified trajectory.
		 */
		std::shared_ptr<StartTrajectory_Response> sStartTrajectory(const std::shared_ptr<StartTrajectory::Request> request, std::shared_ptr<StartTrajectory::Response> response);

		/**
		 * @brief Instructs the drone to take off and hover at the given height.
		 */
		std::shared_ptr<Takeoff_Response> sTakeoff(const std::shared_ptr<Takeoff::Request> request, std::shared_ptr<Takeoff::Response> response);
		
		/**
		 * @brief Lands and subsequently disarms the drone.
		 */
		std::shared_ptr<Land_Response> sLand(const std::shared_ptr<Land::Request> request, std::shared_ptr<Land::Response> response);

		/**
		 * @brief Instructs the drone to move to the given position.
		 */
		std::shared_ptr<GoTo_Response> sGoTo(const std::shared_ptr<GoTo::Request> request, std::shared_ptr<GoTo::Response> response);

		/**
		 * @brief Specifies the trajectory the drone should move along.
		 */
		std::shared_ptr<UploadTrajectory_Response> sUploadTrajectory(const std::shared_ptr<UploadTrajectory::Request> request, std::shared_ptr<UploadTrajectory::Response> response);
		
		/**
		 * @brief Instructs the drone to stop moving along trajectory target points.
		 */
		std::shared_ptr<NotifySetpointsStop_Response> sNotifySetpointsStop(const std::shared_ptr<NotifySetpointsStop::Request> request, std::shared_ptr<NotifySetpointsStop::Response> response);

		/**
		 * @brief Arms the drone, starting the motors.
		 */
		std::shared_ptr<Arm_Response> sArm(const std::shared_ptr<Arm::Request> request, std::shared_ptr<Arm::Response> response);
};
#endif /* CRAZYFLIE_BRIDGE_H */