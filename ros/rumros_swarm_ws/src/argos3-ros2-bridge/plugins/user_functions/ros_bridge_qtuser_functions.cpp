/*
 * ros_bridge_qtuser_functions.cpp
 * Author: Alexander Kassuba
 */

#include "ros_bridge_qtuser_functions.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <variant>
/****************************************/
/****************************************/

using std::placeholders::_1;

CIDQTUserFunctions::CIDQTUserFunctions() {
   try {
      // Retrive loop functions to obtain option and area definitions
      auto& cLoopFns = dynamic_cast<CRosBridgeLoopFunctions&>(
         argos::CSimulator::GetInstance().GetLoopFunctions());

      areas = cLoopFns.GetAreas();
      options = cLoopFns.GetOptions();

      // Parse options to variables for quicker access
      for (Option o : options) {
         if (std::holds_alternative<bool>(o.value))
            LOG << "Option (bool): " << o.name << " = " << std::get<bool>(o.value) << std::endl;
         else if (std::holds_alternative<std::string>(o.value))
            LOG << "Option (str): " << o.name << " = " << std::get<std::string>(o.value) << std::endl;
         else if (std::holds_alternative<int>(o.value))
            LOG << "Option (int): " << o.name << " = " << std::get<int>(o.value) << std::endl;
         else if (std::holds_alternative<double>(o.value))
            LOG << "Option (double): " << o.name << " = " << std::get<double>(o.value) << std::endl;
         
         if (o.name == OPT_DRAW_ROBOT_NAME) {
            b_drawRobotName = std::get<bool>(o.value);
            i_numRobotDrawOptions++;
         } else if (o.name == OPT_DRAW_ROBOT_GROUP) {
            b_drawRobotGroup = std::get<bool>(o.value);
            i_numRobotDrawOptions++;
         }
      }

   } catch (std::bad_cast& e) {
      // Treat execptions as user not having defined loop functions and continue
      return;
   }

   if (b_drawRobotGroup) {
      // Build map of robot IDs to group IDs
      auto& tRoot = argos::CSimulator::GetInstance().GetConfigurationRoot();
      auto& tArena = GetNode(tRoot, "arena");
      argos::TConfigurationNodeIterator it;
      bool err = false;
      for (it = it.begin(&tArena); it != it.end(); ++it) {
         if (it->Value() == "turtlebot3" || it->Value() == "foot-bot") {
            std::string id;
            std::string group_id = "";
            GetNodeAttribute(*it, "id", id);

            try {
               GetNodeAttribute(*it, "group_id", group_id);
            } catch (CARGoSException& ex) {
               err = true;
            }
            
            robotGroupMap.emplace(id, group_id);
         }
      }

      if (err) {
         b_drawRobotGroup = false;
         i_numRobotDrawOptions--;
         LOG << "Not all robot tags contain 'group_id' attribute. Not rendering group IDs" << std::endl;
      }
   }

   // ===== ROS Init =====    
	// Create a context with the domain ID
   auto context = std::make_shared<rclcpp::Context>();
   rclcpp::InitOptions init_options;
   init_options.set_domain_id(0);
   init_options.auto_initialize_logging(false);  // Prevent multiple logging inits
   context->init(0, nullptr, init_options);

   // Create node with unique name
   std::string node_name = "argos_qtuser_functions_node";
   nodeHandle_ = std::make_shared<rclcpp::Node>(node_name, rclcpp::NodeOptions().context(context));

	// Retrieve and print the actual domain ID
	auto actual_domain_id = context->get_domain_id();
	RCLCPP_INFO(nodeHandle_->get_logger(), "Running on ROS_DOMAIN_ID: %zu", actual_domain_id);

   // Initialize command subscribers
   cmdTopicSubscriber = nodeHandle_ -> create_subscription<std_msgs::msg::String>(
                           "/rumros/cmd_ctrl",
                           10,
                           std::bind(&CIDQTUserFunctions::CmdCallback, this, _1)
                        );

   ackTopicSubscriber = nodeHandle_ -> create_subscription<std_msgs::msg::String>(
                           "/rumros/cmd_ack",
                           10,
                           std::bind(&CIDQTUserFunctions::AckCallback, this, _1)
                        );

   RegisterUserFunction<CIDQTUserFunctions,CFloorEntity>(&CIDQTUserFunctions::Draw);
   RegisterUserFunction<CIDQTUserFunctions,CFootBotEntity>(&CIDQTUserFunctions::Draw);
   RegisterUserFunction<CIDQTUserFunctions,CTurtlebot3Entity>(&CIDQTUserFunctions::Draw);
}

/****************************************/

/**
 * @brief This callback monitors the cmd_ctrl topic of RuMROS to log pending group changes.
 * 
 * @param msg the ROS string message observed on the cmd_ctrl topic
 */
void CIDQTUserFunctions::CmdCallback(const std_msgs::msg::String &msg)
{
    try {
        nlohmann::json command = nlohmann::json::parse(msg.data);

        RCLCPP_INFO(nodeHandle_->get_logger(), "Received CMD: %s", command.dump().c_str());

        // If message is changeGroupCommand, store it
        if (command.contains("type") && command["type"] == 1 && command["receiver"] != "__controller__") {
            std::string uuid = command["uuid"];
            RCLCPP_INFO(nodeHandle_->get_logger(), "Setting pending updates for UUID: %s", uuid.c_str());
            std::lock_guard<std::mutex> lock(m_mutex);
            m_pendingGroupUpdates[command["uuid"]] = command;
        }
    }
    catch (std::exception& e) {
        LOGERR << "Failed to parse command JSON: " << e.what() << std::endl;
    }
}

void CIDQTUserFunctions::AckCallback(const std_msgs::msg::String &msg)
{
   try {
      nlohmann::json ack = nlohmann::json::parse(msg.data);

      RCLCPP_INFO(nodeHandle_->get_logger(), "Received ACK: %s", ack.dump().c_str());

      // If message is ACK
      if (ack.contains("type") && ack["type"] == 0 && ack["receiver"] == "__controller__") {
         std::string uuid = ack["content"];

         RCLCPP_INFO(nodeHandle_->get_logger(), "ACK UUID is: %s", uuid.c_str());

         std::lock_guard<std::mutex> lock(m_mutex);
         m_seenAcks.push_back(uuid);
      }
   }
   catch (std::exception& e) {
      LOGERR << "Failed to parse ACK JSON: " << e.what() << std::endl;
   }
}

/****************************************/

void CIDQTUserFunctions::Draw(CFloorEntity& c_entity) {
   // Make sure ROS node functions by spinning periodically
   rclcpp::spin_some(nodeHandle_);

   {
      std::lock_guard<std::mutex> lock(m_mutex);
      // Check pending command ACKs
      for (size_t i = 0; i < m_seenAcks.size(); ) {
         std::string uuid = m_seenAcks[i];
         auto it = m_pendingGroupUpdates.find(uuid);

         if (it != m_pendingGroupUpdates.end()) {
            nlohmann::json originalCommand = it->second;
            std::string robot_id = originalCommand["receiver"];
            std::string new_group_id = originalCommand["content"]["name"];

            robotGroupMap[robot_id] = new_group_id;

            RCLCPP_INFO(nodeHandle_->get_logger(), "Changing robot %s to group %s", robot_id.c_str(), new_group_id.c_str());

            m_pendingGroupUpdates.erase(it);
            m_seenAcks.erase(m_seenAcks.begin() + i); // erase current
            break;
         }
         i++;
      }
   }

   // Draw area names in their center
   for (const Area& area : areas) {
      double cx = (area.x1 + area.x2) / 2.0;
      double cy = (area.y1 + area.y2) / 2.0;
      DrawText(CVector3(cx, cy, 0.3), area.name.c_str());
   }
}

void CIDQTUserFunctions::draw_robot_info(CEntity& c_entity) {
   // Set positions to draw text
   std::vector<double> drawingPos;
   drawingPos.push_back(0.3);
   if (i_numRobotDrawOptions > 1) {
      drawingPos.push_back(0.6);
   }

   // Draw text
   if (b_drawRobotName) {
      // Draw name
      DrawText(CVector3(0.0, 0.0, drawingPos[drawingPos.size() - 1]),
               c_entity.GetId().c_str()); // Name
      drawingPos.pop_back();
   }

   if (b_drawRobotGroup) {
      // Draw group
      std::string group_id = robotGroupMap.at(c_entity.GetId());

      DrawText(CVector3(0.0, 0.0, drawingPos[drawingPos.size() - 1]),
               group_id.c_str()); // Group
      drawingPos.pop_back();
   }
}

void CIDQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   draw_robot_info(static_cast<CEntity&>(c_entity));
}

void CIDQTUserFunctions::Draw(CTurtlebot3Entity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */

   draw_robot_info(static_cast<CEntity&>(c_entity));
}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "ros_bridge_user_functions")
