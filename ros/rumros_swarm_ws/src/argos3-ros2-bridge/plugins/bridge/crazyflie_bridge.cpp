/*
 * crazyflie_bridge.cpp (TU Dresden)
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
 * and can be actuated directly (currently used) or via software-in-the-loop (SIL)
 * simulation (planned).
 *
 *  Created on: 20 May 2026
 */

#include "crazyflie_bridge.h"
#include "base/ros_bridge_singleton.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <functional>
#include <cfloat>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/logging/argos_colored_text.h>
#include <argos3/core/simulator/simulator.h>
#include <stdexcept>

// Uncomment to enable software-in-the-loop control over the CF
// Currently in development
//#define USE_SIL_COMPUTATION

using namespace std::placeholders;

CrazyflieBridge::CrazyflieBridge() : m_pcSpdAct(NULL), m_pcPosAct(NULL), m_pcLedAct(NULL), m_pcPosSens(NULL) {}

CrazyflieBridge::~CrazyflieBridge()
{

}

void CrazyflieBridge::Init(TConfigurationNode &t_node)
{
    try
    {
        name = GetId();

        // Initialize ROS node only once
        auto &bridge = RosBridgeSingleton::Instance();
        nodeHandle = bridge.node_;

        // Build sensor availability cache
        hasSpeedActuator = HasActuator("quadrotor_speed");
        hasPosActuator = HasActuator("quadrotor_position");
        hasLedsActuator = HasActuator("leds");

        hasPositioningSensor = HasSensor("positioning");
        hasCameraSensor = HasSensor("crazyflie_colored_blob_perspective_camera");

        // Topics
        // Setup sensors and actuators with corresponding ROS publishers and subscribers
        if (hasPositioningSensor) // For now, only positioning, e.g. via flow deck is supported
        {
            m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
            pubPose = nodeHandle->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", 10);
        }

        pubRobotDescription = nodeHandle->create_publisher<std_msgs::msg::String>(name + "/robot_description", rclcpp::QoS(1).transient_local());
        if (hasSpeedActuator)
        {
            m_pcSpdAct = GetActuator<CCI_QuadRotorSpeedActuator>("quadrotor_speed");
            subCmdVel = nodeHandle->create_subscription<Twist>(
                name + "/cmd_vel_legacy", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieBridge::cbCmdVelLegacy, this, _1));
            
            // World space velocity commands require additional positioning sensor
            if (hasPositioningSensor)
            {
                subCmdVelWorld = nodeHandle->create_subscription<VelocityWorld>(
                    name + "/cmd_velocity_world", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieBridge::cbCmdVelWorld, this, _1));
            }
        }
        if (hasPosActuator && hasPositioningSensor)
        {
            m_pcPosAct = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
            subCmdPosition = nodeHandle->create_subscription<Position>(
                name + "/cmd_position", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieBridge::cbCmdPosition, this, _1));
            subCmdHover = nodeHandle->create_subscription<Hover>(
                name + "/cmd_hover", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieBridge::cbCmdHover, this, _1));
            subCmdFullState = nodeHandle->create_subscription<FullState>(
                name + "/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieBridge::cbCmdFullState, this, _1));
        }
        if (hasLedsActuator)
        {
            m_pcLedAct = GetActuator<CCI_LEDsActuator>("leds");
            subCmdLed = nodeHandle->create_subscription<ColorRGBA>(
                name + "/cmd_led", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieBridge::cbCmdLed, this, _1));
        }

        // Setup services
        srvEmergency = nodeHandle->create_service<Empty>(name + "/emergency", std::bind(&CrazyflieBridge::sEmergency, this, _1, _2), rclcpp::ServicesQoS());
        srvArm = nodeHandle->create_service<Arm>(name + "/arm", std::bind(&CrazyflieBridge::sArm, this, _1, _2), rclcpp::ServicesQoS());

        if (hasPosActuator && hasPositioningSensor)
        {
            // Only available with a positioning sensor and controller
            srvStartTrajectory = nodeHandle->create_service<StartTrajectory>(name + "/start_trajectory", std::bind(&CrazyflieBridge::sStartTrajectory, this, _1, _2), rclcpp::ServicesQoS());
            srvTakeoff = nodeHandle->create_service<Takeoff>(name + "/takeoff", std::bind(&CrazyflieBridge::sTakeoff, this, _1, _2), rclcpp::ServicesQoS());
            srvLand = nodeHandle->create_service<Land>(name + "/land", std::bind(&CrazyflieBridge::sLand, this, _1, _2), rclcpp::ServicesQoS());
            srvGoTo = nodeHandle->create_service<GoTo>(name + "/go_to", std::bind(&CrazyflieBridge::sGoTo, this, _1, _2), rclcpp::ServicesQoS());
            srvUploadTrajectory = nodeHandle->create_service<UploadTrajectory>(name + "/upload_trajectory", std::bind(&CrazyflieBridge::sUploadTrajectory, this, _1, _2), rclcpp::ServicesQoS());
            srvSetpointsStop = nodeHandle->create_service<NotifySetpointsStop>(name + "/notify_setpoints_stop", std::bind(&CrazyflieBridge::sNotifySetpointsStop, this, _1, _2), rclcpp::ServicesQoS());
        }

        mode = MODE_IDLE;

        // Time
        cSimulator = &argos::CSimulator::GetInstance();
        timeStep = cSimulator->GetPhysicsEngine("default").GetSimulationClockTick();

        #ifdef USE_SIL_COMPUTATION
        // Setup CF firmware
        firmware = new CFFirmware(m_pcPosSens->GetReading().Position, CFFirmware::ControllerType::CONTROLLER_PID, [this]() {return this->getTime();}); // Use position sensor to get initial simulated position, this makes the sensor required for now
        physics = new CrazyfliePhysics(firmware->getState());
        #endif

        RCLCPP_INFO(nodeHandle->get_logger(), "Initialized Crazyflie controller for %s", name.c_str());
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing Crazyflie", ex);
    }
}

/*****************************************/
/**************** Helpers ****************/
/*****************************************/

void CrazyflieBridge::setLedColor(CColor color)
{
    if (m_pcLedAct)
        m_pcLedAct->SetAllColors(color);
}

inline Real CrazyflieBridge::getTime()
{
    UInt64 unTick = cSimulator->GetSpace().GetSimulationClock();
    return unTick * timeStep;
}

void CrazyflieBridge::moveToSetpoint(CVector3 goal, CRadians yaw, bool relative)
{
    if (m_pcPosSens && m_pcPosAct)
    {
        if (relative)
        {
            m_pcPosAct->SetRelativePosition(goal);
            m_pcPosAct->SetAbsoluteYaw(yaw);
        } else
        {
            m_pcPosAct->SetAbsolutePosition(goal);
            m_pcPosAct->SetAbsoluteYaw(yaw);
        }
    }
}

inline CVector3 rosVec3ToArgosVec3(geometry_msgs::msg::Vector3 rosVec)
{
    return CVector3(rosVec.x, rosVec.y, rosVec.z);
}

inline CVector3 rosPointToArgosVec3(geometry_msgs::msg::Point rosPoint)
{
    return CVector3(rosPoint.x, rosPoint.y, rosPoint.z);
}

inline CVector3 cfPosToArgosVec3(crazyflie_interfaces::msg::Position cfPos)
{
    return CVector3(cfPos.x, cfPos.y, cfPos.z);
}

inline float durationToSec(const builtin_interfaces::msg::Duration& d)
{
    return d.sec + 1e-9f * d.nanosec;
}

inline CVector3 CrazyflieBridge::getQuadPosition()
{
    return currentPos;
}

inline CQuaternion CrazyflieBridge::getQuadOrientation()
{
    if (m_pcPosSens)
    {
        CCI_PositioningSensor::SReading pos = m_pcPosSens->GetReading();
        return pos.Orientation;
    }
    throw new std::runtime_error("Attempted to fetch quad orientation, but no positional sensor exists");
}

inline bool positionReached(CVector3 p1, CVector3 p2, float epsilon=0.01f)
{
    return(p1 - p2).Length() < epsilon;
}

// Polynomial trajectory approximation functions (sample at distance epsilon and return sampled points)
inline float evalPoly(const std::vector<float>& coeffs, float t)
{
    float result = 0.0f;
    for (int i = coeffs.size() - 1; i >= 0; --i)
    {
        result = result * t + coeffs[i];
    }
    return result;
}

inline CVector3 evalPosition(const TrajectoryPolynomialPiece& piece, float t)
{
    return {
        evalPoly(piece.poly_x, t),
        evalPoly(piece.poly_y, t),
        evalPoly(piece.poly_z, t)
    };
}

std::vector<CrazyflieBridge::Setpoint> evalPolynomial(
    const std::vector<TrajectoryPolynomialPiece>& traj,
    float epsilon)
{
    // TODO: test speed-based evaluation to better match real drone
    std::vector<CrazyflieBridge::Setpoint> result;

    if (traj.empty()) return result;

    CVector3 prevPoint{};
    bool firstPoint = true;

    for (const auto& piece : traj)
    {
        float T = durationToSec(piece.duration);

        if (T <= 0.0f)
            continue;

        // Adaptive step (start coarse)
        float dt = T / 100.0f;
        float t = 0.0f;

        int startIndex = result.size() - 1;
        startIndex = std::max<int>(0, startIndex); // Zero if negative 
        int n = 0;
        while (t <= T)
        {
            // Normalize time to [0,1]
            float tau = t / T;

            CVector3 p = evalPosition(piece, tau);

            if (firstPoint)
            {
                result.push_back({p, 0.0, T}); // TODO: implement yaw
                prevPoint = p;
                firstPoint = false;
            }
            else
            {
                float dist = (p - prevPoint).Length();

                if (dist >= epsilon)
                {
                    result.push_back({p, 0.0, T}); // TODO: implement yaw
                    prevPoint = p;

                    // Adjust step size: increase slightly
                    dt *= 1.2f;
                }
                else
                {
                    // Adjust step size: decrease for better resolution
                    dt *= 0.5f;
                }
            }

            // Clamp dt to avoid instability
            dt = std::clamp(dt, T / 10000.0f, T / 10.0f);
            t += dt;
            n++;
        }

        // Divide duration by number of segments written
        for (int i = startIndex; i < startIndex + n; i++)
        {
            result[i].duration /= (float)n;
        }
    }

    return result;
}

/*****************************************/
/****** Callback & Service Handlers ******/
/*****************************************/

void CrazyflieBridge::cbCmdVelLegacy(geometry_msgs::msg::Twist::SharedPtr msg)
{
    // With ARGoS, this is only supported with the speed controller directly
    #ifndef USE_SIL_COMPUTATION
    // Currently, the implementations of velLegacy and velWorld in the firmware are the same
    if (m_pcSpdAct)
    {
        // Only allows for movement in 2D plane at current height
        m_pcSpdAct->SetRotationalSpeed(CRadians(msg->angular.z));
        m_pcSpdAct->SetLinearVelocity(rosVec3ToArgosVec3(msg->linear));
    }
    else
        LOGERR << "[" << name << "]" << " has no speed controller, cannot execute cmd_vel_legacy" << std::endl;
    #endif

    #ifdef USE_SIL_COMPUTATION
    // Original Python CF SIL does not support this behavior
    //LOGERR << "[" << name << "]" << " cmdVelWorld is [not implemented]" << std::endl;

    // Experimental implementation for ARGoS
    firmware->cmdVelLegacy(rosVec3ToArgosVec3(msg->linear), msg->angular.z);
    #endif
}

void CrazyflieBridge::cbCmdVelWorld(crazyflie_interfaces::msg::VelocityWorld::SharedPtr msg)
{
    #ifndef USE_SIL_COMPUTATION
    // Currently, the implementations of velLegacy and velWorld in the firmware are the same
    if (m_pcSpdAct)
    {
        // Only allows for movement in 2D plane at current height
        m_pcSpdAct->SetRotationalSpeed(CRadians(msg->yaw_rate));
        m_pcSpdAct->SetLinearVelocity(rosVec3ToArgosVec3(msg->vel));
    }
    else
        LOGERR << "[" << name << "]" << " has no speed controller, cannot execute cmd_vel_world" << std::endl;
    #endif

    #ifdef USE_SIL_COMPUTATION
    // Original Python CF SIL does not support this behavior
    //LOGERR << "[" << name << "]" << " cmdVelWorld is [not implemented]" << std::endl;

    // Experimental implementation for ARGoS
    firmware->cmdVelWorld(rosVec3ToArgosVec3(msg->vel), msg->yaw_rate);
    #endif
}

void CrazyflieBridge::cbCmdPosition(crazyflie_interfaces::msg::Position::SharedPtr msg)
{
    #ifndef USE_SIL_COMPUTATION
    if (!m_pcPosAct)
    {
        LOGERR << "[" << name << "]" << " has no position controller, cannot execute cmd_position" << std::endl;

    }
    // tPrevSetpoint is valid
    if (setpoint != nullptr && getTime() - tPrevSetpoint >= tSetpointTimeout)
    {
        // Timeout exceeded, failsafe
        LOG << "[" << name << "]" << " position timeout occurred, failsafing" << std::endl;
        mode = MODE_FAILSAFE;
        movingToSetpoint = false;
        setpoint = nullptr;
        tPrevSetpoint = -1.0;
        return;
    }
    tPrevSetpoint = getTime();
    if (setpoint == nullptr)
        setpoint = new Setpoint();
    setpoint->position = CVector3(msg->x, msg->y, msg->z);
    setpoint->yaw = msg->yaw; // Duration not part of message, ignored
    mode = MODE_SETPOINT_STREAM;
    movingToSetpoint = true;
    
    #endif

    #ifdef USE_SIL_COMPUTATION
    // Requires positionController in firmware, currently not implemented in
    // Python and here
    LOGERR << "[" << name << "]" << " cmd_position is [not implemented]" << std::endl;
    #endif
}

void CrazyflieBridge::cbCmdHover(crazyflie_interfaces::msg::Hover::SharedPtr msg)
{
    #ifndef USE_SIL_COMPUTATION
    // Not defined for Gazebo and difficult to implement with ARGoS, as it
    // specifies both a height to hover at and velocities and turn rates
    // to move in cardinal directions, locked at the specified height.
    // This means, both positional and speed controllers of the underlying
    // library are required simultaneously, which is not currently supported
    LOGERR << "[" << name << "]" << " cmd_hover is not supported" << std::endl;
    #endif

    #ifdef USE_SIL_COMPUTATION
    // Original Python CF SIL does not support this behavior
    //LOGERR << "[" << name << "]" << " cmdHover is [not implemented]" << std::endl;

    // Experimental implementation for ARGoS
    // TODO: Other methods use wrapper with ARGoS types, implement here as well
    firmware->setHoverSetpoint(&firmware->setpoint, msg->vx, msg->vy, msg->yaw_rate, msg->z_distance);
    #endif
}

void CrazyflieBridge::cbCmdFullState(crazyflie_interfaces::msg::FullState::SharedPtr msg)
{
    // CQuaternion quat;
    // quat.Set(msg->pose.orientation.w,
    //          msg->pose.orientation.x,
    //          msg->pose.orientation.y,
    //          msg->pose.orientation.z);
    
    // CRadians x_angle, y_angle, z_angle;
    // quat.ToEulerAngles(z_angle, y_angle, x_angle);
    
    #ifndef USE_SIL_COMPUTATION
    if (m_pcSpdAct)
    {
        // Use only given speed values
        m_pcSpdAct->SetLinearVelocity(rosVec3ToArgosVec3(msg->twist.linear));
        m_pcSpdAct->SetRotationalSpeed(CRadians(msg->twist.angular.z));
    } else if (m_pcPosAct)
    {
        // Use only given position and yaw
        m_pcPosAct->SetAbsolutePosition(rosPointToArgosVec3(msg->pose.position));
        m_pcPosAct->SetAbsoluteYaw(CRadians(msg->twist.angular.z));
    } else
        LOGERR << "[" << name << "]" << " has no controller, cannot execute cmd_full_state" << std::endl;
    #endif
    
    #ifdef USE_SIL_COMPUTATION
    firmware->cmdFullState(rosPointToArgosVec3(msg->pose.position),
                           rosVec3ToArgosVec3(msg->twist.linear),
                           rosVec3ToArgosVec3(msg->acc),
                           msg->twist.angular.z,
                           //z_angle.GetValue(),
                           rosVec3ToArgosVec3(msg->twist.angular));
    #endif

}

void CrazyflieBridge::cbCmdLed(std_msgs::msg::ColorRGBA::SharedPtr msg)
{
    LOG << "Setting color LED deck to (" << msg->r << ", " << msg->g << ", " << msg->b << ")" << std::endl;
    setLedColor(CColor(msg->r, msg->g, msg->b, msg->a));
}

std::shared_ptr<Empty_Response> CrazyflieBridge::sEmergency(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response)
{
    LOG << "[" << name << "]" << " emergency()" << std::endl;

    // An exemplary native Crazyflie plugin implementation without SIL controller might look like this:
    #ifndef USE_SIL_COMPUTATION 
    if (m_pcSpdAct)
    {
        // Set all velocities to 0
        m_pcSpdAct->SetLinearVelocity(CVector3(0,0,0));
        m_pcSpdAct->SetRotationalSpeed(CRadians(0.0));
    } else if (m_pcPosAct)
    {
        // Emergency land
        mode = MODE_FAILSAFE;
        movingToSetpoint = false;
        // Simple stop & hover
        //m_pcPosAct->SetRelativePosition(CVector3(0,0,0);
    } else
        LOGERR << "[" << name << "]" << " has no controller, cannot call emergency()" << std::endl;
    #endif

    #ifdef USE_SIL_COMPUTATION
    LOG << "[" << name << "]" << " emergency stopping is [not implemented]" << std::endl;
    #endif
    return response;
}

std::shared_ptr<StartTrajectory_Response> CrazyflieBridge::sStartTrajectory(const std::shared_ptr<StartTrajectory::Request> request, std::shared_ptr<StartTrajectory::Response> response)
{
    LOG << "[" << name << "]" << " startTrajectory(id=" << request->trajectory_id
        << ", timescale=" << request->timescale
        << ", reversed=" << request->reversed
        << ", relative=" << request->relative
        << ", group_mask=" << request->group_mask << ")" << std::endl;

    #ifndef USE_SIL_COMPUTATION
        mode = MODE_TRAJECTORY; // Functionality handled in ControlStep()
        movingToSetpoint = false; // Ensures setpoint is only set once in loop
    #endif

    #ifdef USE_SIL_COMPUTATION
    firmware->startTrajectory(request->trajectory_id, request->timescale, request->reversed, request->relative, true, request->group_mask);
    #endif

    return response;
}

std::shared_ptr<Takeoff_Response> CrazyflieBridge::sTakeoff(const std::shared_ptr<Takeoff::Request> request, std::shared_ptr<Takeoff::Response> response)
{
    float d = (float)request->duration.sec + (float)(request->duration.nanosec / 1e+9);
    LOG << "[" << name << "]" << " takeoff(height=" << request->height
        << "m, duration=" << d
        << " s, group_mask=" << request->group_mask << ")" << std::endl;

    #ifndef USE_SIL_COMPUTATION
    if (mode != MODE_IDLE)
    {
        LOGERR << "[" << name << "]" << " can only takeoff() in idle state" << std::endl;
        return response;
    }

    CVector3 pos = getQuadPosition();
    CVector3 target = CVector3(pos.GetX(), pos.GetY(), pos.GetZ() + request->height);
    takeoffLandData = {pos, target, request->height, d};
    mode = MODE_TAKEOFF; // Functionality handled in ControlStep()
    moveToSetpoint(takeoffLandData.goalPosition);
    #endif

    #ifdef USE_SIL_COMPUTATION
    firmware->takeoff(request->height, d, request->group_mask);
    #endif

    return response;
}

std::shared_ptr<Land::Response> CrazyflieBridge::sLand(const std::shared_ptr<Land::Request> request, std::shared_ptr<Land::Response> response)
{
    float d = (float)request->duration.sec + (float)(request->duration.nanosec / 1e+9);
    LOG << "[" << name << "]" << " land(height=" << request->height
        << "m, duration=" << d
        << " s, group_mask=" << request->group_mask << ")" << std::endl;

    #ifndef USE_SIL_COMPUTATION
    if (mode == MODE_IDLE || mode == MODE_FAILSAFE)
    {
        LOGERR << "[" << name << "]" << " cannot land() in idle/failsafe states" << std::endl;
        return response;
    }

    CVector3 pos = getQuadPosition();
    // Real drone shuts off motors upon reaching target height, in simulator it switches to MODE_IDLE
    CVector3 target = CVector3(pos.GetX(), pos.GetY(), pos.GetZ() - request->height);
    takeoffLandData = {pos, target, -request->height, d};
    mode = MODE_LAND; // Functionality handled in ControlStep()
    if (request->height != 0)
        moveToSetpoint(takeoffLandData.goalPosition);
    // If height is 0, controlStep has to move downwards in small increments instead of going to a set goal
    #endif

    #ifdef USE_SIL_COMPUTATION
    firmware->land(request->height, d, request->group_mask);
    #endif

    return response;
}

std::shared_ptr<GoTo_Response> CrazyflieBridge::sGoTo(const std::shared_ptr<GoTo::Request> request, std::shared_ptr<GoTo::Response> response)
{
    float d = (float)request->duration.sec + (float)(request->duration.nanosec / 1e+9);
    LOG << "[" << name << "]" << " goTo(position=" << request->goal.x << "," << request->goal.y << "," << request->goal.z
        << " m, yaw=" << request->yaw
        << ", duration=" << d
        << " s, relative=" << request->relative
        << ", group_mask=" << request->group_mask << ")" << std::endl;
    
    #ifndef USE_SIL_COMPUTATION
    if (mode == MODE_IDLE || mode == MODE_FAILSAFE)
    {
        LOGERR << "[" << name << "]" << " cannot goTo() in idle/failsafe states" << std::endl;
        return response;
    }
    // GoTo has no mode, as it simply comples the move unless interruped,
    // no action required in the main loop
    moveToSetpoint(rosPointToArgosVec3(request->goal), CRadians(request->yaw), request->relative);
    #endif

    #ifdef USE_SIL_COMPUTATION
    firmware->goTo(rosPointToArgosVec3(request->goal), request->yaw, d, request->relative, request->group_mask);
    #endif

    return response;
}

std::shared_ptr<UploadTrajectory_Response> CrazyflieBridge::sUploadTrajectory(const std::shared_ptr<UploadTrajectory::Request> request, std::shared_ptr<UploadTrajectory::Response> response)
{
    LOG << "[" << name << "]" << " uploadTrajectory(id=" << request->trajectory_id
        << ", n_pieces=" << request->pieces.size() << ")" << std::endl;

    #ifndef USE_SIL_COMPUTATION
    // No guards, as upload is possible at any time
    // Compute polynomial setpoints
    trajSetpoints = evalPolynomial(request->pieces, 0.05f); // 'Medium resolution' evaluation
    iTrajSetpoint = 0;
    #endif

    #ifdef USE_SIL_COMPUTATION
    // Build polynomial from request pieces
    CFFirmware::TrajectoryPolynomial polynomial;
    for (auto rp : r->pieces)
    {
        CFFirmware::TrajectoryPolynomialPiece pp;
        pp.poly_x = rp.poly_x;
        pp.poly_y = rp.poly_y;
        pp.poly_z = rp.poly_z;
        pp.poly_yaw = rp.poly_yaw;
        pp.duration = (float)rp.duration.sec + (float)(rp.duration.nanosec / 1e+9);;
        polynomial.push_back(pp);
    }
    firmware->uploadTrajectory(r->trajectory_id, r->piece_offset, polynomial);
    #endif

    return response;
}

std::shared_ptr<NotifySetpointsStop_Response> CrazyflieBridge::sNotifySetpointsStop(const std::shared_ptr<NotifySetpointsStop::Request> request, std::shared_ptr<NotifySetpointsStop::Response> response)
{
    // Not implemented as it is not relevant to simulated functionality
    LOG << "[" << name << "]" << " notifySetpointsStop is not supported" << std::endl;
    return response;
}

std::shared_ptr<Arm_Response> CrazyflieBridge::sArm(const std::shared_ptr<Arm::Request> request, std::shared_ptr<Arm::Response> response)
{
    // The concept of arming is currently not supported by simulated Crazyflies,
    // as it is not vital to the simulation.
    // For now, set color of LED, based on arm status
    if (request->arm)
        setLedColor(CColor::RED); 
    else
        setLedColor(CColor::YELLOW);

    return response;
}

/******************************************/
/************** Control Loop **************/
/******************************************/
void CrazyflieBridge::ControlStep()
{
    rclcpp::spin_some(nodeHandle);

    if (m_pcPosSens)
    {
        CCI_PositioningSensor::SReading pos = m_pcPosSens->GetReading();
        currentPos = pos.Position;
        currentOrientation = pos.Orientation;
    } else
        throw new std::runtime_error("Attempted to fetch quad position, but no positional sensor exists");

    // === Publish ROS messages ===
    // Pose
    if (pubPose)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        rclcpp::Time curr_time = nodeHandle->get_clock()->now();
		pose_msg.header.stamp = curr_time;
		pose_msg.header.frame_id = "world"; // 'world' is also the default of Crazyswarm, when used with a FlowDeck

        // Obtain reading manually to reduce overhead
		CCI_PositioningSensor::SReading pos = m_pcPosSens->GetReading();

		// Write current pose
		pose_msg.pose.position.x = pos.Position.GetX();
		pose_msg.pose.position.y = pos.Position.GetY();
		pose_msg.pose.position.z = pos.Position.GetZ();
		pose_msg.pose.orientation.x = pos.Orientation.GetX();
		pose_msg.pose.orientation.y = pos.Orientation.GetY();
		pose_msg.pose.orientation.z = pos.Orientation.GetZ();
		pose_msg.pose.orientation.w = pos.Orientation.GetW();

		pubPose->publish(pose_msg);
    }

    #ifndef USE_SIL_COMPUTATION
    // TODO: Crazyflie plugin does not support setting velocity, adjust the controller so it does
    //  Interfaces in this class have already been adapted - setpoints have individual durations
    //  from which speeds have to be computed
    // TODO: Add yaw to setpoints
    switch (mode)
    {
    case MODE_TAKEOFF:
    // Move up until target reached
    if (m_pcPosSens && positionReached(takeoffLandData.goalPosition, currentPos))
    {
        // Arrived
        mode = MODE_HOVER;
    }
    break;
    case MODE_LAND:
    if (takeoffLandData.height == 0)
    {
        // Move down in small increments
        moveToSetpoint(CVector3(currentPos.GetX(), currentPos.GetY(), currentPos.GetZ() - descentRate));
    }

    else if (m_pcPosSens && positionReached(takeoffLandData.goalPosition, currentPos))
    {
        // Target reached
        // takeoffLandData.height == 0.0 prevents this from triggering
        // immediately at the starting position, when height is set to 0
        // Arrived, disarm
        setLedColor(CColor::YELLOW); // Disarmed color
        prevPos = nullptr;
        landInitTime = -1;
        mode = MODE_IDLE;
    }
    
    if (m_pcPosSens && prevPos && prevPos->GetZ() != takeoffLandData.initialPosition.GetZ() && positionReached(*prevPos, currentPos))
    {
        // Should be moving, but position did not change for 2 seconds, consider
        // this a terrain hit and thus landed
        setLedColor(CColor::YELLOW); // Disarmed color
        prevPos = nullptr;
        landInitTime = -1;
        mode = MODE_IDLE;
    }

    // Assign previous position initially
    if (landInitTime < 0)
        landInitTime = cSimulator->GetSpace().GetSimulationClock();

    // Assign previous position every two seconds
    if (m_pcPosSens && cSimulator->GetSpace().GetSimulationClock() >= landInitTime + static_cast<UInt32>(2.0 / timeStep))
    {
        if (!prevPos)
            prevPos = new CVector3(currentPos);
        else
            *prevPos = currentPos;
        landInitTime = cSimulator->GetSpace().GetSimulationClock();
    }
    break;
    case MODE_SETPOINT_STREAM:
        moveToSetpoint(setpoint->position, CRadians(setpoint->yaw));
        // No position reached check, as STREAM mode does not care
        break;
    case MODE_FAILSAFE:
        // Real drone turns off propellers and drops, simulated drone enters failsafe state and lands
        // TODO: implement
        break;
    case MODE_TRAJECTORY:
        if (m_pcPosSens && m_pcPosAct && !movingToSetpoint)
        {
            // First setpoint, start
            movingToSetpoint = true;
            moveToSetpoint(trajSetpoints[0].position);
        }
        if (m_pcPosSens && m_pcPosAct && positionReached(trajSetpoints[iTrajSetpoint].position, currentPos))
        {
            // Arrived at setpoint, fly to next setpoint
            if ((unsigned int)iTrajSetpoint == trajSetpoints.size() - 1) // Last setpoint, stop
                break;
            moveToSetpoint(trajSetpoints[++iTrajSetpoint].position);
        }
        break;
    default:
        break;
    }
    #endif

    #ifdef USE_SIL_COMPUTATION

    // Execute control loop
    Real t = getTime();
    CFFirmware::Action action = firmware->executeController();
    CFFirmware::State state = physics->step(action, t - prevTime);
    firmware->setState(state);

    // Actuate
    m_pcSpdAct->SetLinearVelocity(state.velocity);
    m_pcSpdAct->SetRotationalSpeed(CRadians(state.omega.GetZ()));
    prevTime = t;

    // Perform physics simulation
    // float mass = 0.034f;     // kg
    // float armLength = 0.046; // m
    // float arm = 0.707106781f * armLength;
    // float t2t = 0.006f;      // Thrust to torque ratio
    // float g = 9.81;
    // // CMatrix<3,3> I = CMatrix<3,3>(); // Inertia matrix, see Ian Snider (https://iansnider.com/files/Snider_Ian_ESE4481_Final.pdf)
    // // I.Set(new Real[9]{
    // //     16.571710e-6, 0.830806e-6, 0.718277e-6,
    // //     0.830806e-6, 16.655602e-6, 1.800197e-6,
    // //     0.718277e-6, 1.800197e-6, 29.261652e-6
    // // }); // kg * m^2
    // // Simplified model, diagonal only
    // float I[3] = {16.571710e-6, 16.655602e-6, 29.261652e-6};
    // CMatrix<4,4> B0 = CMatrix<4,4>();
    // B0.Set(new Real[16]{
    //     1, 1, 1, 1,
    //     -arm, -arm, arm, arm,
    //     -arm, arm, arm, -arm,
    //     -t2t, t2t, -t2t, t2t
    // });
    
    // Update CF state

    // Compute SIL Action and convert to high-level values ARGoS can work with
    // This requires motor thrust to be converted to a linear velocity and yaw rate
    // and is a simplified model which does not take complex physical effects
    // like ground effect, etc. into account, as the simulation is
    // designed to be simplistic with a sufficient degree of realism, in order
    // to make simulation of larger MRS feasible. More information on how the firmware
    // internally computes parameters can be found at
    // https://iansnider.com/files/Snider_Ian_ESE4481_Final.pdf as well as the 
    // crazyflie-firmware GitHub repo.
    // For more information on physical parameters, see
    // https://github.com/bitcraze/crazyflie-firmware/blob/master/src/platform/interface/platform_defaults_cf2.h.
    //============================
    // // Physical drone parameters
    // constexpr float MASS = 0.029f;         // kg (depends on sensor setup, 29g is the base weight with only a flow deck)
    // constexpr float ARM_LENGTH = 0.046f;   // meters
    // CMatrix<3,3> I = CMatrix<3,3>();       // Inertia matrix, see Ian Snider
    // I.Set(new Real[9]{
    //     16.571710e-6, 0.830806e-6, 0.718277e-6,
    //     0.830806e-6, 16.655602e-6, 1.800197e-6,
    //     0.718277e-6, 1.800197e-6, 29.261652e-6
    // });

    // CFFirmware::Action action = firmware->executeController();
    // //CFFirmware::State cfState = firmware->getSetpoint();

    // // Motor PWM thrust is computed by the firmware.
    // // Using real world flight data, the Crazyswarm
    // // devs derived a method to compute thrust (pwmToForce)
    // float f1 = firmware->pwmToForce(action.pwm[0]);
    // float f2 = firmware->pwmToForce(action.pwm[1]);
    // float f3 = firmware->pwmToForce(action.pwm[2]);
    // float f4 = firmware->pwmToForce(action.pwm[3]);

    // //LOG << "PWM:" << std::endl;
    // //LOG << action.pwm[0] << ", " << action.pwm[1] << ", " << action.pwm[2] << ", " << action.pwm[3] << std::endl;
    // //LOG << "FORCE:" << std::endl;
    // //LOG << f1 << ", " << f2 << ", " << f3 << ", " << f4 << std::endl;

    // // Apply Snider's mixer matrix
    // float fZ = f1 + f2 + f3 + f4;
    // float tauRoll = f1 + f2 - f3 - f4;
    // float tauPitch = -f1 + f2 + f3 - f4;
    // float tauYaw = f1 - f2 + f3 - f4;

    // // Actuate
    // CVector3 vel = CVector3(0.0, 0.0, fZ);
    // float yaw = firmware->control.yaw;

    // m_pcSpdAct->SetLinearVelocity(vel);
    // m_pcSpdAct->SetRotationalSpeed(CRadians(0.0));
    // LOG << "TQ:" << vel.GetX() << ", " << vel.GetY() << ", " << vel.GetZ() << std::endl;
    // //LOG << "YAW:" << yaw << std::endl;

    // // In lower level modes, use setpoint
    // // m_pcSpdAct->SetLinearVelocity(cfState.velocity);
    // // m_pcSpdAct->SetRotationalSpeed(CRadians(cfState.omega.GetZ()));
    #endif
}

REGISTER_CONTROLLER(CrazyflieBridge, "crazyflie_ros_controller")
