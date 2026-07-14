#include "robot_bridge_base.h"
#include "ros_bridge_singleton.h"
#include <argos3/core/utility/logging/argos_log.h>

RobotBridgeBase::RobotBridgeBase() : halfBaseline(0.08), wheelRadius(0.033) {}

RobotBridgeBase::~RobotBridgeBase() {
    Destroy();
}

void RobotBridgeBase::Init(TConfigurationNode& t_node) {
    try {
        // Parse XML parameters
        robot_id_ = GetId(); // Get robot ID from ARGoS
        GetNodeAttributeOrDefault(t_node, "multiple_domains", multiple_domains_, multiple_domains_);
        GetNodeAttributeOrDefault(t_node, "nodes_per_domain", nodes_per_domain_, nodes_per_domain_);
        GetNodeAttributeOrDefault(t_node, "ros_domain_id", domain_id_, domain_id_);
	    GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount_, stopWithoutSubscriberCount_);

        // Initialize ROS node only once
        auto& bridge = RosBridgeSingleton::Instance();
        nodeHandle_ = bridge.node_;

        if (HasSensor("positioning")) {
            m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
            posePublisher_ = nodeHandle_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + robot_id_ + "/pose", 1);
        }

        if (HasActuator("differential_steering")) {
            m_pcWheels = GetActuator< CCI_DifferentialSteeringActuator >("differential_steering");
            cmdVelSubscriber_ = nodeHandle_->create_subscription<geometry_msgs::msg::Twist>(
                "/" + robot_id_ + "/cmd_vel", 1,
                std::bind(&RobotBridgeBase::cmdVelCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(nodeHandle_->get_logger(), "Initialized RobotBridgeBase for %s", robot_id_.c_str());
    }
    catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing RobotBridgeBase", ex);
    }
}

void RobotBridgeBase::ControlStep() {
    // Default: subclasses implement actual control logic.
    // For the sensors and actuators specified in this base class
    // behavior is handled here
    rclcpp::spin_some(nodeHandle_);

    // Perform subclass behavior via hook
    ControlStepHook();

	// Publish pose
	if (m_pcPositioning && posePublisher_){
        geometry_msgs::msg::PoseStamped pose_msg;
        rclcpp::Time curr_time = nodeHandle_->get_clock()->now();
		pose_msg.header.stamp = curr_time;
		pose_msg.header.frame_id = "pose";

		CCI_PositioningSensor::SReading pos = m_pcPositioning->GetReading();

		// Write current pose
		pose_msg.pose.position.x = pos.Position.GetX();
		pose_msg.pose.position.y = pos.Position.GetY();
		pose_msg.pose.position.z = pos.Position.GetZ();
		pose_msg.pose.orientation.x = pos.Orientation.GetX();
		pose_msg.pose.orientation.y = pos.Orientation.GetY();
		pose_msg.pose.orientation.z = pos.Orientation.GetZ();
		pose_msg.pose.orientation.w = pos.Orientation.GetW();

		posePublisher_->publish(pose_msg);
	}

    // Update velocity
    // If we haven't heard from the subscriber in a while, set the speed to zero.
    if (stepsSinceCallback_ > stopWithoutSubscriberCount_) {
        leftSpeed_ = 0;
        rightSpeed_ = 0;
    } else {
        stepsSinceCallback_++;
    }

    if (m_pcWheels) {
        m_pcWheels->SetLinearVelocity(leftSpeed_, rightSpeed_);
    }
}

void RobotBridgeBase::Destroy() {
    if (nodeHandle_) {
        RCLCPP_INFO(nodeHandle_->get_logger(), "Shutting down node for %s", robot_id_.c_str());
        nodeHandle_.reset();
    }
}

void RobotBridgeBase::cmdVelCallback(const geometry_msgs::msg::Twist& twist) {
   	double v = twist.linear.x;		// Forward linear velocity
	double omega = twist.angular.z; // Rotational (angular) velocity
	double L = halfBaseline * 2;	// Distance between wheels (wheelbase)
	double R = wheelRadius;		// Wheel radius

	// Calculate left and right wheel speeds using differential drive kinematics
	leftSpeed_ = (v - (L / 2) * omega) / R;
	rightSpeed_ = (v + (L / 2) * omega) / R;

	stepsSinceCallback_ = 0;
}
