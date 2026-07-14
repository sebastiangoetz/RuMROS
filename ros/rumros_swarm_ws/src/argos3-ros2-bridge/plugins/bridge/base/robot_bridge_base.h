#ifndef ROBOT_BASE_H_
#define ROBOT_BASE_H_

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <memory>

using namespace argos;

/**
 * @brief This is the base class for concrete mobile robot controllers, from which
 * controllers specific to a certain robot type can inherit. Provides common sensors,
 * parameters and  actuators, as well as methods to control the robot with each step
 * of the simulation.
 * 
 * Each different robot type has to be added as a separate library in CMakeLists.txt
 * and imported accordingly in the ARGoS configuration file.
 * 
 */
class RobotBridgeBase : public CCI_Controller {
protected:
    // === Robot identification and ROS domain setup ===
    std::string robot_id_;
    bool multiple_domains_{false};
    int nodes_per_domain_{10};
    int domain_id_{0};

    // === ROS2 node & context ===
    std::shared_ptr<rclcpp::Node> nodeHandle_;
    rclcpp::Context::SharedPtr context_;

    // === Common publishers & subscribers ===
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;

    // === Common ARGoS sensors & actuators
    CCI_PositioningSensor* m_pcPositioning{nullptr};
    CCI_DifferentialSteeringActuator* m_pcWheels{nullptr};


    // === Management variables ===
    /*
	 * The following variables are used as parameters for the
	 * algorithm. You can set their value in the <parameters> section
	 * of the XML configuration file, under the
	 * <controllers><argos_ros_bot_controller> section.
	*/
    double halfBaseline = 0.0;
    double wheelRadius = 0.0;

    Real leftSpeed_{0.0};
    Real rightSpeed_{0.0};


    // The number of time steps from the time step of the last callback
	// after which leftSpeed and rightSpeed will be set to zero.  Useful to
	// shutdown the robot after the controlling code on the ROS side has quit.
    int stopWithoutSubscriberCount_{10};

    // The number of time steps since the last callback.
    int stepsSinceCallback_{0};

public:
    RobotBridgeBase();
    virtual ~RobotBridgeBase();

    // === ARGoS lifecycle ===

    /*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_ccw_wander_controller> section.
	*/
    virtual void Init(TConfigurationNode& t_node) override;

    /*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	*/
    virtual void ControlStep() override;

    virtual void ControlStepHook() {};

	// Called to cleanup what done by Init() when the experiment finishes
    virtual void Destroy() override;

    // === ROS-related variables and callbacks ===

	// The callback method for getting new commanded speed on the cmd_vel topic
    virtual void cmdVelCallback(const geometry_msgs::msg::Twist& twist);
};

#endif // ROBOT_BASE_H_