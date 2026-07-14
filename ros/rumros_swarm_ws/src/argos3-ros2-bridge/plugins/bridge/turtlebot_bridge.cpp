/*
 * turtlebot_bridge.cpp (based on footbot_bridge.cpp)
 * Author: Alexander Kassuba
 */

/* Include the controller definition */
#include "turtlebot_bridge.h"
#include <sstream>
#include <vector>

using namespace std;
using namespace rumros_msgs::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

TurtlebotBridge::TurtlebotBridge() = default;
TurtlebotBridge::~TurtlebotBridge() = default;

void TurtlebotBridge::Init(TConfigurationNode& t_node){
	// Call base class to init ROS node, context, pose publisher, cmd_vel subscriber
    RobotBridgeBase::Init(t_node);

	// Override physical parameters provided by base class
	// The following constant values were copied from the Turtlebot3 simulations Github
	// repository, see https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/main/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
	halfBaseline = 0.08;
	wheelRadius = 0.033;
	
	// === Actuators ===
	if (HasActuator("leds")){
		stringstream cmdLedTopic;
		cmdLedTopic << "/" << robot_id_ << "/cmd_led";
		m_pcLEDs = GetSensor < CCI_LEDsActuator >("leds");
		cmdLedSubscriber_ = nodeHandle_ -> create_subscription<Led>(cmdLedTopic.str(), 1, std::bind(&TurtlebotBridge::cmdLedCallback, this, _1));
	}

	// === Sensors ===
	if (HasSensor("ground")){
		stringstream groundTopic;
		groundTopic << "/" << robot_id_ << "/groundList";
		m_pcGround = GetSensor < CCI_GroundSensor >("ground");
		groundListPublisher_ = nodeHandle_ -> create_publisher<LightList>(groundTopic.str(), 1);
	}

	if (HasSensor("turtlebot3_proximity")){
		stringstream proxTopic;
		proxTopic << "/" << robot_id_ << "/proximityList";
		m_pcProximity = GetSensor < CCI_Turtlebot3ProximitySensor >("turtlebot3_proximity");
		promixityListPublisher_ = nodeHandle_ -> create_publisher<ProximityList>(proxTopic.str(), 1);
	}

	if (HasSensor("light")){
		stringstream lightTopic;
		lightTopic << "/" << robot_id_ << "/lightList";
		m_pcLight = GetSensor < CCI_LightSensor >("light");
		lightListPublisher_ = nodeHandle_ -> create_publisher<LightList>(lightTopic.str(), 1);
	}

	if (HasSensor("turtlebot3_lidar")){
		stringstream lidarTopic;
		lidarTopic 			<< "/" << robot_id_ << "/lidarScan";
		m_pcLidar 		= GetSensor < CCI_Turtlebot3LIDARSensor >("turtlebot3_lidar");
		lidarScanPublisher_ = nodeHandle_ -> create_publisher<LidarScan>(lidarTopic.str(), 1);
	}

	// Enable LEDs if equipped
	if (HasActuator("leds")){
		m_pcLEDs->SetSingleColor(12, CColor::RED);
	}
}

void TurtlebotBridge::cmdLedCallback(const Led& ledColor){
	if ( ledColor.color == "red" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::RED);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::RED);
		}
	}
	 else if ( ledColor.color == "yellow" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::YELLOW);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::YELLOW);
		}
	}
	else if ( ledColor.color == "green" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::GREEN);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::GREEN);
		}
	}
	else if ( ledColor.color == "magenta" ){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::MAGENTA);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::MAGENTA);
		}
	}
	else if ( ledColor.color == "black"){
		if (ledColor.mode == "ALL"){
			m_pcLEDs->SetAllColors(CColor::BLACK);
		}
		else if (ledColor.mode == "SINGLE"){
			m_pcLEDs->SetSingleColor(ledColor.index, CColor::BLACK);
		}
	}
}

void TurtlebotBridge::ControlStepHook() {
	// Process ground sensor and publish readings
	if (m_pcGround && groundListPublisher_){
		const std::vector<Real>& tGroundReads = m_pcGround->GetReadings();
		LightList groundList; // Ground sensors are also an array of light sensors
		groundList.n = tGroundReads.size();
		for (int i = 0; i < groundList.n; ++i) {
			Light light;
			light.value = tGroundReads[i];
			light.angle = -1.0; // Not implemented, as default sensor is used and turtlebot variation is not implemented yet
			groundList.lights.push_back(light);
		}

		groundListPublisher_ -> publish(groundList);
	}
	
	// Process proximity sensor and publish readings
	if (m_pcProximity && promixityListPublisher_){
		const CCI_Turtlebot3ProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
		ProximityList proxList;
		proxList.n = tProxReads.size();
		for (int i = 0; i < proxList.n; ++i) {
			Proximity prox;
			prox.value = tProxReads[i].Value;
			prox.angle = tProxReads[i].Angle.GetValue();
			proxList.proximities.push_back(prox);
		}

		promixityListPublisher_ -> publish(proxList);
	}	
	
	// Process light sensor and publish readings
	if (m_pcLight && lightListPublisher_){
		const std::vector<Real>& tLightReads = m_pcLight->GetReadings();
		LightList lightList;
		lightList.n = tLightReads.size();
		for (int i = 0; i < lightList.n; ++i) {
			Light light;
			light.value = tLightReads[i];
			light.angle = -1.0; // Not implemented, as default sensor is used and turtlebot variation is not implemented yet
			lightList.lights.push_back(light);
		}

		lightListPublisher_ -> publish(lightList);
	}

	// Process lidar sensor and publish readings
	if (m_pcLidar && lidarScanPublisher_){
        LidarScan lidarScan;
        lidarScan.n = m_pcLidar->GetNumReadings();
        for(int i = 0; i < lidarScan.n; ++i) {
            lidarScan.datapoints.push_back(m_pcLidar->GetReading(i));
        }
		lidarScanPublisher_ -> publish(lidarScan);
	}
}

/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(TurtlebotBridge, "turtlebot_ros_controller")
