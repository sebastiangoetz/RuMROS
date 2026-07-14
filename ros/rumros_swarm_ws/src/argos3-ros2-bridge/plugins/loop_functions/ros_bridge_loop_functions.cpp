/*
 * ros_bridge_loop_functions.cpp
 * Author: Alexander Kassuba
 */

#include "ros_bridge_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CRosBridgeLoopFunctions::CRosBridgeLoopFunctions() : m_pcFloor(NULL) {}

/****************************************/
/****************************************/

bool is_enabled(TConfigurationNode& node, bool _default) {
    // Checks if <node enabled="true/false"> property exists
    // and returns its value
    bool enabled = false;
    try {
        GetNodeAttributeOrDefault(node, "enabled", enabled, _default);
    } catch(const CARGoSException& ex) {
        LOGERR << "Error parsing 'enabled' attribute: " << ex.what() << std::endl;
    }
    return enabled;
}

bool is_enabled(TConfigurationNode& node) {
    // Shortcut wrapper where default is false
    return is_enabled(node, false);
}

void CRosBridgeLoopFunctions::Init(TConfigurationNode& t_node) {
    try {
        m_pcFloor = &GetSpace().GetFloorEntity();

        // Parse area config
        TConfigurationNode& tAreasNode = GetNode(t_node, "areas");

        // Get <area> information
        TConfigurationNodeIterator it;
        for (it = it.begin(&tAreasNode); it != it.end(); ++it) {
            if (it->Value() == "area") {
                std::string strName;

                Area area;
                GetNodeAttribute(*it, "name", area.name);
                GetNodeAttribute(*it, "x1", area.x1);
                GetNodeAttribute(*it, "y1", area.y1);
                GetNodeAttribute(*it, "x2", area.x2);
                GetNodeAttribute(*it, "y2", area.y2);
                
                m_vecAreas.push_back(area);
            }
        }
    }
    catch (CARGoSException& ex) {
        LOG << "Could not parse loop_functions <area> configuration, skipping.." << std::endl;
    }

    TConfigurationNode tOptionsNode;
    try {
        // Get other custom parameters
        tOptionsNode = GetNode(t_node, "options");
    }
    catch (CARGoSException& ex) {
        LOG << "Could not parse loop_functions <options> configuration, skipping.." << std::endl;
    }

    int c = 0;
    try {
        TConfigurationNodeIterator it;
        for (it = it.begin(&tOptionsNode); it != it.end(); ++it) {
            c++;
            Option opt;
            opt.name = it->Value();
            if (it->Value() == OPT_DRAW_ROBOT_NAME) {
                opt.value = is_enabled(*it);
            } else if (it->Value() == OPT_DRAW_ROBOT_GROUP) {
                opt.value = is_enabled(*it);
            } // Add more options here as needed (opt.value can be of any type declared in header)
            m_vecOptions.push_back(opt);
        }
    } catch (CARGoSException& ex) {
        LOG << "Error parsing child tag " << c << " in loop_functions <options>" << std::endl;
    }

    #ifdef PERFLOG
        perf_log.open(log_path, std::ios::out | std::ios::app);

        if (!perf_log.is_open()) {
            throw std::runtime_error("Could not open performance log file");
        }
    #endif
}

/****************************************/
/****************************************/

void CRosBridgeLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

void CRosBridgeLoopFunctions::Destroy() {

}

/****************************************/
/****************************************/

/**
 * @brief Retrieves the color of the simulated floor at the given position. This is
 * called by simulator code for every position on the grid of the floor.
 * 
 * This implementation draws areas specified in the configuration file on the floor
 * of the arena. Colors are picked in a round-robin fashion from a pre-defined array
 * of available colors.
 * 
 * @param c_position_on_plane the position on the plane to retrive the color for
 * @return The computed CColor of the floor at the given position if inside a
 * specified area, or white if not in a specified area.
 */
CColor CRosBridgeLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    static const std::vector<CColor> areaColors = {
        CColor::RED, CColor::GREEN, CColor::BLUE,
        CColor::YELLOW, CColor::MAGENTA, CColor::CYAN,
        CColor::ORANGE, CColor::PURPLE, CColor::BROWN
    };

    for (size_t i = 0; i < m_vecAreas.size(); ++i) {
        const Area& area = m_vecAreas[i];
        if (c_position_on_plane.GetX() >= area.x1 &&
            c_position_on_plane.GetX() <= area.x2 &&
            c_position_on_plane.GetY() >= area.y1 &&
            c_position_on_plane.GetY() <= area.y2) {
            return areaColors[i % areaColors.size()]; // Wrap if more areas than colors
        }
    }

    // Default color if not in any area
    return CColor::WHITE;
}

/****************************************/
/****************************************/

/**
 * @brief This method is called before every simulation step.
 * Re-draws the floor and optionally logs performance data.
 * 
 */
void CRosBridgeLoopFunctions::PreStep() {
    // Trigger re-draw
    m_pcFloor->SetChanged();

    #ifdef PERFLOG
        if (!initialized) {
            last_time = std::chrono::steady_clock::now();
            initialized = true;
        } else {
            auto now = std::chrono::steady_clock::now();
            auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
            last_time = now;

            perf_log << "LOOP_TIME " << diff_ms << std::endl;

            if (++write_counter % 100 == 0) {
                write_counter = 0;
                perf_log.flush();
            }
        }
    #endif
}

std::vector<Area> CRosBridgeLoopFunctions::GetAreas()
{
    return m_vecAreas;
}

std::vector<Option> CRosBridgeLoopFunctions::GetOptions()
{
    return m_vecOptions;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CRosBridgeLoopFunctions, "ros_bridge_loop_functions")