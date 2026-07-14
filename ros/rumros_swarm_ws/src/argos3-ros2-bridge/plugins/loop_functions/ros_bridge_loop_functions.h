/*
 * ros_bridge_loop_functions.h
 * Author: Alexander Kassuba
 */

#ifndef ROS_BRIDGE_LOOP_FUNCTIONS
#define ROS_BRIDGE_LOOP_FUNCTIONS

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <variant>
#include <map>
#include <filesystem>
#include <chrono>
#include <iostream>

#define OPT_DRAW_ROBOT_NAME "draw_robot_name"
#define OPT_DRAW_ROBOT_GROUP "draw_robot_group"

// Uncomment to enable performance logging
//#define PERFLOG

using namespace argos;

struct Area {
    std::string name;
    double x1, x2, y1, y2;
};

struct Option {
   std::string name;
   std::variant<std::string, int, double, bool> value;
};

/**
 * @brief Implements the loop functions of the bridge, which run with every simulation
 * tick globally and independently of robots, as a separate library that can be imported
 * in ARGoS configuration files.
 * 
 * Currently used to draw areas defined in the runtime model and written to <options> in
 * ARGoS configuration files on the floor of the simulation.
 * 
 */
class CRosBridgeLoopFunctions : public CLoopFunctions {

public:

   CRosBridgeLoopFunctions();
   virtual ~CRosBridgeLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

   // Area parameters
   std::vector<Area> GetAreas();

   // Other custom options
   std::vector<Option> GetOptions();

private:
   std::vector<Area> m_vecAreas;
   std::vector<Option> m_vecOptions;
   CFloorEntity* m_pcFloor;

   std::unordered_map<std::string, std::string> m_groupMap;

   // Performance logging
   #ifdef PERFLOG
      int write_counter = 0;
      bool initialized = false;
      std::chrono::steady_clock::time_point last_time;
      std::filesystem::path log_path = std::filesystem::canonical(std::filesystem::current_path()
         .parent_path().parent_path()) / "perflogs" / "argos.log";
      std::ofstream perf_log;
   #endif
};

#endif // ROS_BRIDGE_LOOP_FUNCTIONS
