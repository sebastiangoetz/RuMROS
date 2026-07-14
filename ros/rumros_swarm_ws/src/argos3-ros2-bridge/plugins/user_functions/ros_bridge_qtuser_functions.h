/*
 * ros_bridge_qtuser_functions.h
 * Author: Alexander Kassuba
 */

#ifndef ROS_BRIDGE_QTUSER_FUNCTIONS_H
#define ROS_BRIDGE_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/turtlebot3/simulator/turtlebot3_entity.h>
#include "../loop_functions/ros_bridge_loop_functions.h"
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <iostream>
#include <sstream>
#include <cuchar>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace argos;

/**
 * @brief Implements a QT user functions library that can be linked in an ARGoS
 * configuration file. Used to draw robot and group names above robots and area
 * names on the floor. It also monitors the cmd_ctrl topic of RuMROS for group
 * switch messages and corrsponding ACKs, in order to update displayed groups
 * dynamically at runtime. 
 * 
 */
class CIDQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CIDQTUserFunctions();

   virtual ~CIDQTUserFunctions() {}

   void Draw(CFloorEntity& c_entity);
   void Draw(argos::CFootBotEntity& c_entity);
   void Draw(argos::CTurtlebot3Entity& c_entity);

   // ROS subscribers for commands
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdTopicSubscriber;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ackTopicSubscriber;

   void CmdCallback(const std_msgs::msg::String& msg);
   void AckCallback(const std_msgs::msg::String& msg);

private:
   std::shared_ptr<rclcpp::Node> nodeHandle_;
   std::vector<Area> areas;
   std::vector<Option> options;

   // For tracking groups (robot id -> group id)
   std::mutex m_mutex;
   std::unordered_map<std::string, std::string> robotGroupMap;

   // For tracking pending ChangeGroupCommands (uuid -> ChangeGroupCommand JSON)
   std::unordered_map<std::string, nlohmann::json> m_pendingGroupUpdates; 
   std::vector<std::string> m_seenAcks; 

   // Option value shortcuts for better performance
   int i_numRobotDrawOptions = 0;
   bool b_drawRobotName = false;
   bool b_drawRobotGroup = false;

   void draw_robot_info(CEntity& c_entity);
};

#endif
