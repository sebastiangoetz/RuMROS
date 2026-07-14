#ifndef CF_FIRMWARE_H
#define CF_FIRMWARE_H

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/matrix/matrix.h>
#include <vector>
#include <map>
#include <functional>
#include <stdexcept>
#include <cstdlib>

// CF firmware functions are compiled as a C library
extern "C"
{
    #include "planner.h"
    #include "stabilizer_types.h"
    #include "controller_pid.h"
    #include "controller_mellinger.h"
    #include "controller_brescianini.h"
    #include "pptraj.h"
    #include "math3d.h"
    #include "power_distribution.h"
}

/**
 * @brief Interface between ARGoS3 and the CF firmware for software-in-the-loop (SIL)
 * simulation. It is modeled after Crazyswarm2's Python implementation for Gazebo:
 * https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie_sim/crazyflie_sim/crazyflie_sil.py.
 * Exposes all internals directly for reuse by other classes. Currently the ARGoS nodes run
 * ROS control loops, which are way too slow compared to the rates the firmware actually
 * executes on device. Therefore, this method does not work properly for controlling simulated
 * drones for now.
 * 
 */
class CFFirmware
{
    public:
    enum Mode
    {
        MODE_IDLE,
        MODE_HIGH_POLY,
        MODE_LOW_FULLSTATE,
        MODE_HOVER,
        MODE_LOW_POSITION,
        MODE_LOW_VELOCITY,
        MODE_EMERGENCY_STOP
    } mode;

    enum ControllerType
    {
        CONTROLLER_NONE,
        CONTROLLER_PID,
        CONTROLLER_MELLINGER,
        CONTROLLER_BRESCIANINI
    } controllerType;

    struct TrajectoryPolynomialPiece
    {
        std::vector<float> poly_x = std::vector<float>();
        std::vector<float> poly_y = std::vector<float>();
        std::vector<float> poly_z = std::vector<float>();
        std::vector<float> poly_yaw = std::vector<float>();
        float duration = 0.0;
    };

    struct State
    {
        CVector3 position;
        CVector3 velocity;
        CQuaternion quat;
        CVector3 omega;
    };

    struct Action
    {
        float pwm[4];
        float rpm[4];

        float& M1() { return rpm[0]; }
        float& M2() { return rpm[1]; }
        float& M3() { return rpm[2]; }
        float& M4() { return rpm[3]; }
    };

    typedef std::vector<TrajectoryPolynomialPiece> TrajectoryPolynomial;
    std::map<int, piecewise_traj*> trajectories;

    planner* cfPlanner;

    CVector3 initialPosition;
    uint8_t groupMask;

    // Previous state for HL commander
    vec cmdHlPos;
    vec cmdHlVel;
    float cmdHlYaw;

    setpoint_t setpoint;
    state_t state; 
    sensorData_t sensors;
    control_t control;
    motors_thrust_uncapped_t motorsThrustUncapped;
    motors_thrust_pwm_t motorsThrustPwm;

    controllerMellinger_t* controlMellinger;

    std::function<float()> timeFunc;

    using ControllerPidFunc = void (*)(control_t*, const setpoint_t*, const sensorData_t*, const state_t*, stabilizerStep_t);
    using ControllerMellingerFunc = void (*)(controllerMellinger_t*, control_t*, const setpoint_t*, const sensorData_t*, const state_t*, stabilizerStep_t);

    ControllerPidFunc controllerPidFunc = nullptr;
    ControllerMellingerFunc controllerMellingerFunc = nullptr;

    /**
     * @brief Initializes the CF firmware in idle state.
     * 
     * @param initialPosition The iniial position of this CF
     * @param controllerType The type of controller to use
     * @param timeFunc A function returning time in seconds as a float (e.g. 0.1 for 100ms, 2.0 for 2 seconds)
     */
    CFFirmware(CVector3 initialPosition, ControllerType controllerType, std::function<float()> timeFunc)
    {
        this->initialPosition = initialPosition;
        this->controllerType = controllerType;
        mode = MODE_IDLE;
        this->timeFunc = timeFunc;

        cfPlanner = new planner();
        plan_init(cfPlanner);
        
        cmdHlPos = mkvec(initialPosition.GetX(), initialPosition.GetY(), initialPosition.GetZ());
        cmdHlVel = vzero();
        cmdHlYaw = 0.0;

        setpoint = setpoint_t();

        // Initialize to prevent teleportation in MODE_IDLE in simulator
        setpoint.position.x = cmdHlPos.x;
        setpoint.position.y = cmdHlPos.y;
        setpoint.position.z = cmdHlPos.z;
        setpoint.velocity.x = 0;
        setpoint.velocity.y = 0;
        setpoint.velocity.z = 0;
        setpoint.acceleration.x = 0;
        setpoint.acceleration.y = 0;
        setpoint.acceleration.z = 0;
        setpoint.attitude.roll = 0;
        setpoint.attitude.pitch = 0;
        setpoint.attitude.yaw = 0;
        CQuaternion q = CQuaternion();
        setpoint.attitudeQuaternion.w = q.GetW();
        setpoint.attitudeQuaternion.x = q.GetX();
        setpoint.attitudeQuaternion.y = q.GetY();
        setpoint.attitudeQuaternion.z = q.GetZ();
        setpoint.attitudeQuaternion.q0 = q.GetW();
        setpoint.attitudeQuaternion.q1 = q.GetX();
        setpoint.attitudeQuaternion.q2 = q.GetY();
        setpoint.attitudeQuaternion.q3 = q.GetZ();

        state = state_t();
        state.position.x = cmdHlPos.x;
        state.position.y = cmdHlPos.y;
        state.position.z = cmdHlPos.z;
        state.velocity.x = 0;
        state.velocity.y = 0;
        state.velocity.z = 0;
        state.acc.x = 0;
        state.acc.y = 0;
        state.acc.z = 0;
        state.attitude.roll = 0;
        state.attitude.pitch = -0;
        state.attitude.yaw = 0;

        sensors = sensorData_t();
        sensors.gyro.x = 0;
        sensors.gyro.y = 0;
        sensors.gyro.z = 0;

        control = control_t();
        motorsThrustUncapped = motors_thrust_uncapped_t();
        motorsThrustPwm = motors_thrust_pwm_t();

        // Set up controller
        switch (controllerType)
        {
        case CONTROLLER_NONE:
            //controllerFunc = nullptr;
            break;
        case CONTROLLER_PID:
            controllerPidInit();
            controllerPidFunc = controllerPid;
            break;
        case CONTROLLER_MELLINGER:
            controlMellinger = new controllerMellinger_t();
            controllerMellingerInit(controlMellinger);
            controllerMellingerFunc = controllerMellinger;
            break;
        case CONTROLLER_BRESCIANINI:
            controllerBrescianiniInit();
            controllerPidFunc = controllerBrescianini;
            break;
        default:
            break;
        }
    }

    /**
     * @brief Determines whether the quad is in a state deemed flyable.
     * 
     * @return true if the quad can fly,
     * @return false otherwise
     */
    inline bool canFly()
    {
        // Simple mode check for now
        return mode != MODE_EMERGENCY_STOP;
    }

    /**
     * @brief Sets the group mask for this CF.
     * 
     * @param groupMask The numeric mask to set
     */
    inline void setGroupMask(uint8_t groupMask)
    {
        this->groupMask = groupMask;
    }

    /**
     * @brief Determines whether this CF is part of the given group or not.
     * 
     * @param groupMask The numeric mask of the group
     * @return true if part of the group,
     * @return false otherwise
     */
    inline bool isGroup(uint8_t groupMask)
    {
        return groupMask == 0 || (this->groupMask & groupMask) > 0;
    }

    /**
     * @brief Converts an ARGoS3 vector to a CF firmware-internal vector.
     * 
     * @param v The CVector3 to convert
     * @return The resulting vec
     */
    inline vec vecFromArgos(CVector3& v)
    {
        return mkvec(v.GetX(), v.GetY(), v.GetZ());
    }

    /**
     * @brief Converts a CF firmware-internal vector to an ARGoS3 vector.
     * 
     * @param v The vector to convert
     * @return The resulting CVector3
     */
    inline CVector3 vecToArgos(vec& v)
    {
        return CVector3(v.x, v.y, v.z);
    }

    /**
     * @brief Converts an ARGoS3 vector to a CF firmware-internal point.
     * 
     * @param v The CVector3 to convert
     * @return The resulting point_t
     */
    inline point_t pointFromArgos(CVector3& v)
    {
        point_t p;
        p.x = v.GetX();
        p.y = v.GetY();
        p.z = v.GetZ();
        return p;
    }

    /**
     * @brief Converts a CF firmware-internal point to an ARGoS3 vector.
     * 
     * @param v The point to convert
     * @return The resulting CVector3
     */
    inline CVector3 pointToArgos(point_t& v)
    {
        return CVector3(v.x, v.y, v.z);
    }

    /**
     * @brief Helper for allocating memory for a CF firmware-internal trajectory
     * polynomial representation.
     * 
     * @param size The size to allocate
     * @return A pointer to the allocated memory
     */
    inline struct poly4d* poly4d_malloc(int size)
    {
        return (struct poly4d*)malloc(sizeof(struct poly4d) * size);
    }

    /**
     * @brief Retrieves a CF firmware-internal polynomial piece representation
     * of the trajectory at the given index.
     * 
     * @param pp The polynomial
     * @param i The index to retrieve
     * @return The piecewise polynomial in CF firmware representation
     */
    inline struct poly4d* piecewise_get(struct piecewise_traj *pp, int i)
    {
        return &pp->pieces[i];
    }

    /**
     * @brief Sets a 4D-polynomial coefficient to the given value.
     * 
     * @param poly The polynomial in CF firmware representation 
     * @param dim The dimensional index of the polynomial to set
     * @param coef The coefficient index to set
     * @param val The value to set
     */
    inline void poly4d_set(struct poly4d *poly, int dim, int coef, float val)
    {
        poly->p[dim][coef] = val;
    }

    /**
     * @brief Evaluates a given polynomial. This is equivalent to numpy's polyval.
     * 
     * @param p The polynomial to evaluate, specified as an array of coefficients
     * @param n The number of coefficients
     * @param x The position to evaluate the polynomial at
     * @return The result of the evaluation (y)
     */
    inline static float polyEval(const float* p, int n, float x)
    {
        float y = 0.0f;
        for (int i = 0; i < n; ++i)
        {
            y = y * x + p[i];
        }
        return y;
    }

    /**
     * @brief Computes the motor RPM from a given PWM value.
     * 
     * @param pwm The PWM value
     * @return The computed RPM value
     */
    inline float pwmToRpm(float pwm)
    {
        if (pwm < 10000.0f)
        {
            return 0.0f;
        }

        // p = [a, b]
        const float p[2] = {3.26535711e-01f, 3.37495115e+03f};

        return polyEval(p, 2, pwm);
    }

    /**
     * @brief Computes the force from a given PWM value. Polyfit using data and
     * scripts from https://github.com/IMRCLab/crazyflie-data-collection.
     * 
     * @param pwm The PWM value
     * @return The computed force
     */
    inline static float pwmToForce(float pwm)
    {
        const float p[3] = {
            1.71479058e-09f,
            8.80284482e-05f,
            -2.21152097e-01f
        };

        float force_g = polyEval(p, 3, pwm);
        float force_N = force_g * 9.81f / 1000.0f;

        return std::max(force_N, 0.0f);
    }

    /**
     * @brief Computes the force from a given RPM value. Polyfit using data and
     * scripts from https://github.com/IMRCLab/crazyflie-data-collection.
     * Adapted from https://github.com/IMRCLab/crazyswarm2/blob/6bc135f6abbe37e9dd26159268fec41e6f6f0fea/crazyflie_sim/crazyflie_sim/backend/np.py
     * 
     * @param pwm The RPM value
     * @return The computed force
     */
    inline static float rpmToForce(float rpm)
    {
        const float p[3] = {
            2.55077341e-08f,
            -4.92422570e-05f,
            -1.51910248e-01f
        };

        float force_g = polyEval(p, 3, rpm);
        float force_N = force_g * 9.81f / 1000.0f;

        return std::max(force_N, 0.0f);
    }

    /**
     * @brief Helper to copy a firmware-internal vector.
     * 
     * @param v The vector to copy
     * @return The copy of the given vector
     */
    inline vec copySVec(vec3_s v)
    {
        return mkvec(v.x, v.y, v.z);
    }

    /**
     * @brief Helper to copy a firmware-internal vector.
     * 
     * @param v The vector to copy
     * @return The copy of the given vector
     */
    inline vec copySVec(vec v)
    {
        return mkvec(v.x, v.y, v.z);
    }

    /**
     * @brief Converts the firmware setpoint to an ARGoS3-compatible, equivalent state.
     * 
     * @param setpoint The setpoint
     * @return State The resulting ARGoS3 state
     */
    State setpointToArgosState(setpoint_t* setpoint)
    {
        auto pos = pointToArgos(setpoint->position);
        auto vel = pointToArgos(setpoint->velocity);
        auto acc = pointToArgos(setpoint->acceleration);
        CVector3 omega = CVector3
        (
            radians(setpoint->attitude.roll),
            radians(setpoint->attitude.pitch),
            radians(setpoint->attitude.yaw)
        );

        CQuaternion quat;
        if (setpoint->mode.quat == modeDisable)
        {
            // Quat mode disabled, compute
            // Compute rotation based on differential flatness
            CVector3 thrust = acc + CVector3(0.0f, 0.0f, 9.81f);
            //CVector3 thrust = acc; // Gravity handled by ARGoS physics engine
            CVector3 z_body = thrust.Normalize();
            Real yaw = radians(setpoint->attitude.yaw);
            CVector3 x_world(std::cos(yaw), std::sin(yaw), 0.0f);

            // Norm mathematically not needed. This addresses numerical issues to ensure R is orthogonal
            CVector3 y_body = CVector3(z_body).CrossProduct(x_world).Normalize();
            CVector3 x_body = CVector3(y_body).CrossProduct(z_body).Normalize();

            // np.column_stack equivalent
            Real r00 = x_body.GetX(); Real r01 = y_body.GetX(); Real r02 = z_body.GetX();
            Real r10 = x_body.GetY(); Real r11 = y_body.GetY(); Real r12 = z_body.GetY();
            Real r20 = x_body.GetZ(); Real r21 = y_body.GetZ(); Real r22 = z_body.GetZ();

            // Bar-Itzhack matrix -> quaternion conversion
            Real K[4][4];

            K[0][0] = r00 - r11 - r22;
            K[0][1] = r01 + r10;
            K[0][2] = r02 + r20;
            K[0][3] = r21 - r12;

            K[1][0] = r01 + r10;
            K[1][1] = r11 - r00 - r22;
            K[1][2] = r12 + r21;
            K[1][3] = r02 - r20;

            K[2][0] = r02 + r20;
            K[2][1] = r12 + r21;
            K[2][2] = r22 - r00 - r11;
            K[2][3] = r10 - r01;

            K[3][0] = r21 - r12;
            K[3][1] = r02 - r20;
            K[3][2] = r10 - r01;
            K[3][3] = r00 + r11 + r22;

            // Get largest Eigenvector
            Real q[4] = {1, 0, 0, 0};  // Initial guess

            for (int iter = 0; iter < 10; ++iter) {
                Real q_new[4] = {0, 0, 0, 0};

                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        q_new[i] += K[i][j] * q[j];
                    }
                }

                // Normalize
                Real norm = std::sqrt(q_new[0]*q_new[0] +
                                    q_new[1]*q_new[1] +
                                    q_new[2]*q_new[2] +
                                    q_new[3]*q_new[3]);

                for (int i = 0; i < 4; ++i) {
                    q[i] = q_new[i] / norm;
                }
            }

            // Map result to quaternion
            // Bar-Itzhack yields [x, y, z, w], ARGoS takes [w, x, y, z]
            quat.Set(q[3], q[0], q[1], q[2]);
            quat.Normalize();
        }
        else
        {
            // Quat mode enabled, use the provided one
            quat.Set(
                setpoint->attitudeQuaternion.w,
                setpoint->attitudeQuaternion.x,
                setpoint->attitudeQuaternion.y,
                setpoint->attitudeQuaternion.z
            );
        }

        return State{pos, vel, quat, omega};
    }

    /**
     * @brief Initiates an emergency stop, by stopping all motors as soon as processed.
     * In the firmware, this is implemented via a CRTP supervisor, simplified here
     * to a simple zero velocity command.
     * 
     */
    void emergencyStop()
    {
        mode = MODE_EMERGENCY_STOP;
        cmdVelWorld(CVector3::ZERO, 0.0f);
    }

    /**
     * @brief Sets the setpoint for hovering at a given height with the set x and y
     * velocities to move in the plane, as well as a yawRate for turning. This
     * method is experimental and may not work as intended. TODO: test and adjust code & doc
     * 
     * @param setpoint The setpoint to modify
     * @param vx The velocity in x direction of the quad (positive - forward, negative - backward)?
     * @param vy The velocity in y direction of the quad (positive - right, negative - left)?
     * @param yawRate The rate at which the quad turns in plane (clockwise, radians)?
     * @param zDistance The absolute height to hover at
     */
    void setHoverSetpoint(setpoint_t* setpoint, float vx, float vy, float yawRate, float zDistance)
    {
        if (!canFly())
            return;

        mode = MODE_HOVER;
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->mode.z = modeAbs;
        setpoint->position.z = zDistance;

        setpoint->mode.yaw = modeVelocity;
        setpoint->velocity.x = vx;
        setpoint->velocity.y = vy;

        setpoint->attitudeRate.yaw = yawRate;

        setpoint->velocity_body = true;
    }

    /**
     * @brief Instructs the firmware controller to take off.
     * 
     * @param targetHeight The height to fly up to
     * @param duration The duration of the move
     * @param groupMask Gate variable, disabling behavior if this CF is not part of the group
     * @return The integer result variable as returned by the firmware controller
     */
    int takeoff(float targetHeight, float duration, uint8_t groupMask=0)
    {
        if (!canFly())
            return -2;

        if (!isGroup(groupMask))
            return -1;
        
        mode = MODE_HIGH_POLY;
        return plan_takeoff(cfPlanner, cmdHlPos, cmdHlYaw, targetHeight, 0.0f, duration, timeFunc());
    }

    /**
     * @brief Instructs the firmware controller to land.
     * 
     * @param targetHeight The height to go down from (set to zero to move down until stopped by geometry)
     * @param duration The duration of the move
     * @param groupMask Gate variable, disabling behavior if this CF is not part of the group
     * @return The integer result variable as returned by the firmware controller
     */
    int land(float targetHeight, float duration, uint8_t groupMask=0)
    {
        if (!canFly())
            return -2;

        if (!isGroup(groupMask))
            return -1;
        
        mode = MODE_HIGH_POLY;
        return plan_land(cfPlanner, cmdHlPos, cmdHlYaw, targetHeight, 0.0f, duration, timeFunc());
    }

    /**
     * @brief Instructs the firmware controller to move to a given position.
     * 
     * @param goal The position to move to
     * @param yaw The final yaw orientation the CF should have upon arrival
     * @param duration The duration of the move
     * @param relative Indicates whether the given position is relative or not
     * @param groupMask Gate variable, disabling behavior if this CF is not part of the group
     * @return The integer result variable as returned by the firmware controller
     */
    int goTo(CVector3 goal, float yaw, float duration, bool relative=false, uint8_t groupMask=0)
    {
        if (!canFly())
            return -2;

        if (!isGroup(groupMask))
            return -1;
        
        if (mode != MODE_HIGH_POLY)
            throw std::runtime_error("goTo is not yet supported from low level modes");
        mode = MODE_HIGH_POLY;
        return plan_go_to(cfPlanner, relative, false, vecFromArgos(goal), yaw, duration, timeFunc());
    }

    /**
     * @brief Hands a trajectory over to the firmware controller.
     * 
     * @param trajectoryId A numeric identifier for this trajectory, required for calls to startTrajectory()
     * @param pieceOffset The offset into the polynomial piece array [unused] 
     * @param polynomial The array of polynomial pieces
     */
    void uploadTrajectory(int trajectoryId, int pieceOffset, TrajectoryPolynomial& polynomial)
    {
        auto traj = new piecewise_traj();
        traj->t_begin = 0;
        traj->timescale = 1.0;
        traj->shift = mkvec(0, 0, 0);
        traj->n_pieces = polynomial.size();
        traj->pieces = poly4d_malloc(traj->n_pieces);
        for (int i = 0; i < traj->n_pieces; i++)
        {
            auto piece = polynomial[i];
            auto fwPiece = piecewise_get(traj, i);
            fwPiece->duration = piece.duration;

            for (int j = 0; j < 8; j++)
            {
                poly4d_set(fwPiece, 0, j, piece.poly_x[j]);
                poly4d_set(fwPiece, 1, j, piece.poly_y[j]);
                poly4d_set(fwPiece, 2, j, piece.poly_z[j]);
                poly4d_set(fwPiece, 3, j, piece.poly_yaw[j]);
            }
            trajectories.emplace(trajectoryId, traj);
        }
    }

    /**
     * @brief Instructs the firmware controller to start moving along a trajectory
     * 
     * @param trajectoryId The numeric identifier assigned to the trajectory on upload
     * @param timescale The timescale of movement
     * @param reverse Whether to traverse the trajectory in reverse
     * @param relativePos Position is computed as relative, if true, absolute otherwise
     * @param relativeYaw Only applies if position is relative; computes yaw as relative if true, absolute otherwise
     * @param groupMask Gate variable, disabling behavior if this CF is not part of the group
     * @return The integer result variable as returned by the firmware controller
     */
    int startTrajectory(int trajectoryId, float timescale=1.0f, bool reverse=false, bool relativePos=true, bool relativeYaw=true, uint8_t groupMask=0)
    {
        if (!canFly())
            return -2;

        if (!isGroup(groupMask))
            return -1;
        
        mode = MODE_HIGH_POLY;
        auto traj = trajectories[trajectoryId];
        traj->t_begin = timeFunc();
        traj->timescale = timescale;
        return plan_start_trajectory(cfPlanner, traj, reverse, relativePos, relativeYaw, cmdHlPos, cmdHlYaw);
    }

    // NOTICE: The notifySetpointsStop method is skipped because it is irrelevant for simulation

    /**
     * @brief Computes the internal state change resulting from a FullState command message.
     * 
     * @param pos The position to set
     * @param vel The velocity to set
     * @param acc The acceleration to set
     * @param yaw The yaw angle to set
     * @param omega The roll, pitch and yaw values of the attitude rate to set [radians]
     */
    void cmdFullState(const CVector3 pos, const CVector3 vel, const CVector3 acc, float yaw, const CVector3 omega)
    {
        if (!canFly())
            return;

        mode = MODE_LOW_FULLSTATE;
        setpoint.position.x = pos.GetX();
        setpoint.position.y = pos.GetY();
        setpoint.position.z = pos.GetZ();
        setpoint.velocity.x = vel.GetX();
        setpoint.velocity.y = vel.GetY();
        setpoint.velocity.z = vel.GetZ();
        setpoint.attitude.yaw = degrees(yaw);
        setpoint.attitudeRate.roll = degrees(omega.GetX());
        setpoint.attitudeRate.pitch = degrees(omega.GetY());
        setpoint.attitudeRate.yaw = degrees(omega.GetZ());
        setpoint.mode.x = modeAbs;
        setpoint.mode.y = modeAbs;
        setpoint.mode.z = modeAbs;
        setpoint.mode.roll = modeDisable;
        setpoint.mode.pitch = modeDisable;
        setpoint.mode.yaw = modeAbs;
        setpoint.mode.quat = modeDisable;
        setpoint.acceleration.x = acc.GetX();
        setpoint.acceleration.y = acc.GetY();
        setpoint.acceleration.z = acc.GetZ();


        cmdHlPos = copySVec(setpoint.position);
        cmdHlVel = copySVec(setpoint.velocity);
        cmdHlYaw = yaw;
    }

    /**
     * @brief Sets the velocity of the Crazyflie. This method is a legacy command
     * implementation and should not be used. Use cmdVelWorld() instead, which is
     * currently identical with this legacy implementation, for more information,
     * see crtp_commander_generic.c (crazyflie-firmware). 
     * 
     * @param velocity The velocity to set
     * @param yawRate The rate at which the quad turns around the z axis (clockwise, radians)?
     */
    void cmdVelLegacy(const CVector3 velocity, float yawRate)
    {
        if (!canFly())
            return;

        // Legacy and world velocity decoders currently have the same implementation
        // See crtp_commander_generic.c (crazyflie-firmware)
        mode = MODE_LOW_VELOCITY;
        setpoint.mode.x = modeVelocity;
        setpoint.mode.y = modeVelocity;
        setpoint.mode.z = modeVelocity;
        setpoint.velocity.x = velocity.GetX();
        setpoint.velocity.y = velocity.GetY();
        setpoint.velocity.z = velocity.GetZ();

        setpoint.mode.yaw = modeVelocity;
        setpoint.attitudeRate.yaw = yawRate;
    }

    void cmdVelWorld(const CVector3 velocity, float yawRate)
    {
        if (!canFly())
            return;

        mode = MODE_LOW_VELOCITY;
        setpoint.mode.x = modeVelocity;
        setpoint.mode.y = modeVelocity;
        setpoint.mode.z = modeVelocity;
        setpoint.velocity.x = velocity.GetX();
        setpoint.velocity.y = velocity.GetY();
        setpoint.velocity.z = velocity.GetZ();

        setpoint.mode.yaw = modeVelocity;
        setpoint.attitudeRate.yaw = yawRate;
    }

    /**
     * @brief Fetches the firmware setpoint and converts it to a state object that ARGoS3 understands.
     * 
     * @return The computed state
     */
    State getSetpoint()
    {
        if (mode == MODE_HIGH_POLY)
        {
            traj_eval ev = plan_current_goal(cfPlanner, timeFunc());
            if (is_traj_eval_valid(&ev))
            {
                setpoint.position.x = ev.pos.x;
                setpoint.position.y = ev.pos.y;
                setpoint.position.z = ev.pos.z;
                setpoint.velocity.x = ev.vel.x;
                setpoint.velocity.y = ev.vel.y;
                setpoint.velocity.z = ev.vel.z;
                setpoint.attitude.yaw = degrees(ev.yaw);
                setpoint.attitudeRate.roll = degrees(ev.omega.x);
                setpoint.attitudeRate.pitch = degrees(ev.omega.y);
                setpoint.attitudeRate.yaw = degrees(ev.omega.z);
                setpoint.mode.x = modeAbs;
                setpoint.mode.y = modeAbs;
                setpoint.mode.z = modeAbs;
                setpoint.mode.roll = modeDisable;
                setpoint.mode.pitch = modeDisable;
                setpoint.mode.yaw = modeAbs;
                setpoint.mode.quat = modeDisable;
                setpoint.acceleration.x = ev.acc.x;
                setpoint.acceleration.y = ev.acc.y;
                setpoint.acceleration.z = ev.acc.z;

                // Python SIL library omits this part
                //setpoint.jerk.x = ev.jerk.x;
                //setpoint.jerk.y = ev.jerk.y;
                //setpoint.jerk.z = ev.jerk.z;

                cmdHlPos = copySVec(ev.pos);
                cmdHlVel = copySVec(ev.vel);
                cmdHlYaw = ev.yaw;
            }
        }
        return setpointToArgosState(&setpoint);
    }

    /**
     * @brief Sets the firmware state based on an ARGoS3-compatible state input.
     * 
     * @param s The ARGoS3-compatible state to set
     */
    void setState(State s)
    {
        this->state.position.x = state.position.x;
        this->state.position.y = state.position.y;
        this->state.position.z = state.position.z;
        this->state.velocity.x = state.velocity.x;
        this->state.velocity.y = state.velocity.y;
        this->state.velocity.z = state.velocity.z;

        // Quaternion to Euler angles
        CRadians x_angle, y_angle, z_angle;
        s.quat.ToEulerAngles(z_angle, y_angle, x_angle);

        Real roll = ToDegrees(x_angle).GetValue();
        Real pitch = ToDegrees(y_angle).GetValue();
        Real yaw = ToDegrees(z_angle).GetValue();

        // CF firmware uses inverted pitch
        pitch = -pitch;

        this->state.attitude.roll  = roll;
        this->state.attitude.pitch = pitch;
        this->state.attitude.yaw   = yaw;

        // Copy quaternion
        this->state.attitudeQuaternion.w = s.quat.GetW();
        this->state.attitudeQuaternion.x = s.quat.GetX();
        this->state.attitudeQuaternion.y = s.quat.GetY();
        this->state.attitudeQuaternion.z = s.quat.GetZ();

        // Omega is part of the sensors, not the state
        this->sensors.gyro.x = degrees(s.omega.GetX());
        this->sensors.gyro.y = degrees(s.omega.GetY());
        this->sensors.gyro.z = degrees(s.omega.GetZ());

        // Python SIL library omits this part
        this->state.acc.x = state.acc.x;
        this->state.acc.y = state.acc.y;
        this->state.acc.z = state.acc.z;
    }

    /**
     * @brief Returns the current state as an ARGoS-compatible object.
     * 
     * @return State The current state
     */
    State getState()
    {
        State s;
        s.position = CVector3(state.position.x, state.position.y, state.position.z);
        s.velocity = CVector3(state.velocity.x, state.velocity.y, state.velocity.z);
        CVector3 omega = CVector3
        (
            radians(state.attitude.roll),
            radians(state.attitude.pitch),
            radians(state.attitude.yaw)
        );
        s.omega = omega;
        s.quat.Set(state.attitudeQuaternion.x,
                   state.attitudeQuaternion.y,
                   state.attitudeQuaternion.z,
                   state.attitudeQuaternion.w
        );
        return s;
    }

    /**
     * @brief Executes the firmware controller given the current state, computing a new motor state for the CF.
     * 
     * @return Action The Action the simulated CF should perform, i.e. computed motor PWM's
     */
    Action executeController()
    {
        if ((controllerType == CONTROLLER_NONE) || (mode == MODE_IDLE))
        {
            return Action{{0, 0, 0, 0},{0, 0, 0, 0}};
        }

        Real time_in_seconds = timeFunc();
        stabilizerStep_t tick = static_cast<stabilizerStep_t>(time_in_seconds * 1000.0f);

        if (controllerType == CONTROLLER_MELLINGER)
        {
            controllerMellingerFunc(controlMellinger, &control, &setpoint, &sensors, &state, tick);
        }
        else
        {
            controllerPidFunc(&control, &setpoint, &sensors, &state, tick); // Signature same for both PID and Brescianini controllers
        }
        return fwControlToAction();
    }

    /**
     * @brief Helper to compute the ARGoS3-compatible Action from firmware-internal control structures.
     * 
     * @return Action The ARGoS3-compatible action
     */
    Action fwControlToAction()
    {
        powerDistribution(&control, &motorsThrustUncapped);
        powerDistributionCap(&motorsThrustUncapped, &motorsThrustPwm);

        Action action;

        action.pwm[0] = motorsThrustPwm.motors.m1;
        action.pwm[1] = motorsThrustPwm.motors.m2;
        action.pwm[2] = motorsThrustPwm.motors.m3;
        action.pwm[3] = motorsThrustPwm.motors.m4;

        action.rpm[0] = pwmToRpm(action.pwm[0]);
        action.rpm[1] = pwmToRpm(action.pwm[1]);
        action.rpm[2] = pwmToRpm(action.pwm[2]);
        action.rpm[3] = pwmToRpm(action.pwm[3]);

        return action;
    }
};

#endif