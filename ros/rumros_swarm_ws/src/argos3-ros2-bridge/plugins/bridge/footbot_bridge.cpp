/*
 * footbot_bridge.cpp (modified to work with RuMROS)
 *
 *  Created on: 20 Jun 2024
 *  Original author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

/* Include the controller definition */
#include "footbot_bridge.h"
#include <sstream>

using namespace std;
using namespace rumros_msgs::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

FootbotBridge::FootbotBridge() = default;
FootbotBridge::~FootbotBridge() = default;

void FootbotBridge::Init(TConfigurationNode& t_node){
	// Call base class to init ROS node, context, pose publisher, cmd_vel subscriber
    RobotBridgeBase::Init(t_node);

	// Override physical parameters provided by base class
	// The following constant values were copied from the argos source tree from
	// the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
	halfBaseline = 0.07;
	wheelRadius = 0.029112741;

	// Sensor init
	if (HasSensor("footbot_light")){
		stringstream lightTopic;
		lightTopic 			<< "/" << robot_id_ << "/lightList";
		m_pcLight  			= GetSensor < CCI_FootBotLightSensor>("footbot_light");
		lightListPublisher_ = nodeHandle_ -> create_publisher<LightList>(lightTopic.str(), 1);

	}

	if (HasSensor("footbot_proximity")){
		stringstream proxTopic;
		proxTopic 			<< "/" << robot_id_ << "/proximityList";
		m_pcProximity 		= GetSensor < CCI_FootBotProximitySensor>("footbot_proximity");
		promixityListPublisher_ = nodeHandle_ -> create_publisher<ProximityList>(proxTopic.str(), 1);
	}

	if (HasSensor("range_and_bearing")){
		stringstream rabTopic;
		rabTopic 			<< "/" << robot_id_ << "/rab";
		m_pcRABS 			= GetSensor < CCI_RangeAndBearingSensor>("range_and_bearing");
		rabPublisher_ 		= nodeHandle_ -> create_publisher<PacketList>(rabTopic.str(), 1);
	}

	if (HasSensor("colored_blob_omnidirectional_camera")){
		stringstream blobTopic;
		blobTopic 			<< "/" << robot_id_ << "/blobList";
		m_pcCamera 			= GetSensor < CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
		blobListPublisher_ 	= nodeHandle_ -> create_publisher<BlobList>(blobTopic.str(), 1);
	}

	// Actuator init
	if (HasActuator("leds")){
		m_pcLEDs = GetActuator< CCI_LEDsActuator >("leds");
		stringstream cmdLedTopic;
		cmdLedTopic 	<< "/" << robot_id_ << "/cmd_led";
		cmdLedSubscriber_ = nodeHandle_ -> create_subscription<Led>(
							cmdLedTopic.str(),
							1,
							std::bind(&FootbotBridge::cmdLedCallback, this, _1)
							);
	}

	if (HasActuator("range_and_bearing")){
		m_pcRABA = GetActuator< CCI_RangeAndBearingActuator >("range_and_bearing");
		stringstream cmdRabTopic;
		cmdRabTopic 	<< "/" << robot_id_ << "/cmd_rab";
		cmdRabSubscriber_ = nodeHandle_ -> create_subscription<Packet>(
							cmdRabTopic.str(),
							1,
							std::bind(&FootbotBridge::cmdRabCallback, this, _1)
							);
	}

	/*
	* Sensor-specific initialization
	*/
	if (HasSensor("colored_blob_omnidirectional_camera")){
	/* Enable camera filtering */
	   m_pcCamera->Enable();
	}

	if (HasActuator("leds")){
		/* Enable */
		m_pcLEDs->SetSingleColor(12, CColor::RED);
	}
}

bool blobComparator(Blob a, Blob b) {
	return a.angle < b.angle;
}

void FootbotBridge::ControlStepHook() {
	/*********************************
	 * Get readings from light sensor
	 *********************************/
	if (m_pcLight && lightListPublisher_){
		const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
		LightList lightList;
		lightList.n = tLightReads.size();
		for (int i = 0; i < lightList.n; ++i) {
			Light light;
			light.value = tLightReads[i].Value;
			light.angle = tLightReads[i].Angle.GetValue();
			lightList.lights.push_back(light);

		}

		lightListPublisher_ -> publish(lightList);
	}
	/***********************************
	 * Get readings from Proximity sensor
	 ***********************************/
	if (m_pcProximity && promixityListPublisher_){
		const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
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
	/**************************************************************
	 * Get readings from Colored Blob Omnidirectional Camera Sensor
	 *************************************************************/
	if (m_pcCamera && blobListPublisher_){
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcCamera->GetReadings();
		BlobList blobList;
		blobList.n = camReads.BlobList.size();
		Blob blob;
		for (int i = 0; i < blobList.n; ++i) {
			//Blob blob;
			stringstream ss;
			ss << camReads.BlobList[i]->Color;
			blob.color = ss.str();
			blob.distance = camReads.BlobList[i]->Distance;

			// Make the angle of the puck in the range [-PI, PI].  This is useful for
			// tasks such as homing in on a puck using a simple controller based on
			// the sign of this angle.
			blob.angle = camReads.BlobList[i]->Angle.GetValue();//.SignedNormalize().GetValue();
			blobList.blobs.push_back(blob);

		}

		// Sort the blob list by angle.  This is useful for the purposes of extracting meaning from
		// the local blob configuration (e.g. fitting a lines to the detected blobs).
		sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

		blobListPublisher_ -> publish(blobList);
	}

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	if (m_pcRABS && rabPublisher_){
		const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRABS->GetReadings();
		PacketList packetList;
		packetList.n = tRabReads.size();
		for (int i = 0; i < packetList.n; ++i) {
			Packet packet;
			packet.range = tRabReads[i].Range;
			packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
			packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

			
			packet.data.push_back(tRabReads[i].Data[0]);
			packet.data.push_back(tRabReads[i].Data[1]);

			packetList.packets.push_back(packet);
			
		}

		rabPublisher_ -> publish(packetList);
	}
}

void FootbotBridge::cmdRabCallback(const Packet& packet){
	m_pcRABA -> SetData(0, packet.data[0]);
	m_pcRABA -> SetData(1, std::stoi( packet.id ));
}

void FootbotBridge::cmdLedCallback(const Led& ledColor){
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
REGISTER_CONTROLLER(FootbotBridge, "footbot_ros_controller")
