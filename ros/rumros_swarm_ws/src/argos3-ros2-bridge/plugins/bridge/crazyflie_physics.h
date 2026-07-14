#include <argos3/core/utility/math/matrix/matrix.h>
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include "cf_firmware.h"

using namespace argos;

/**
 * @brief Integrates a quaternion, see 
 * https://www.ashwinnarayan.com/post/how-to-integrate-quaternions
 * for more details. Mimics rowan.calculus.integrate.
 * 
 * @param q 
 * @param omega_global 
 * @param dt 
 * @return CQuaternion 
 */
CQuaternion IntegrateQuaternion(const CQuaternion& q,
                                   const CVector3& omega_global,
                                   Real dt) {

      // Magnitude of angular velocity
      Real omega_norm = omega_global.Length();

      CQuaternion dq;

      if(omega_norm < 1e-8) {
         // Small-angle approximation:
         // dq ≈ [1, 0.5 * omega * dt]
         dq.Set(1.0,
                0.5 * omega_global.GetX() * dt,
                0.5 * omega_global.GetY() * dt,
                0.5 * omega_global.GetZ() * dt);
      }
      else {
         // Full axis-angle conversion
         Real theta = omega_norm * dt;
         CVector3 axis = omega_global / omega_norm;

         dq.FromAngleAxis(CRadians(theta), axis);
      }

      // IMPORTANT:
      // omega_global => left-multiply
      CQuaternion q_new = dq * q;

      // Normalize to avoid drift
      q_new.Normalize();

      return q_new;
   }

class CrazyfliePhysics
{
    private:
    
    float mass = 0.034f;     // kg
    float armLength = 0.046f; // m
    float arm = 0.707106781f * armLength;
    float t2t = 0.006f;      // Thrust to torque ratio
    float g = 9.81f;
    //float g = 0.0f; // Handled by ARGoS
    // CMatrix<3,3> I = CMatrix<3,3>(); // Inertia matrix, see Ian Snider (https://iansnider.com/files/Snider_Ian_ESE4481_Final.pdf)
    // I.Set(new Real[9]{
    //     16.571710e-6, 0.830806e-6, 0.718277e-6,
    //     0.830806e-6, 16.655602e-6, 1.800197e-6,
    //     0.718277e-6, 1.800197e-6, 29.261652e-6
    // }); // kg * m^2
    // Simplified model, diagonal only
    CMatrix<3,3> I;
    CVector3 invI;
    CMatrix<4,4> B0;

    CFFirmware::State state;

    public:
    CrazyfliePhysics(CFFirmware::State state)
    {
        B0 = CMatrix<4,4>();
        B0.Set(new Real[16]{
            1, 1, 1, 1,
            -arm, -arm, arm, arm,
            -arm, arm, arm, -arm,
            -t2t, t2t, -t2t, t2t
        });

        this->state = state;
        I = CMatrix<3,3>(new Real[9]{
            16.571710e-6, 0, 0,
            0, 16.655602e-6, 0,
            0, 0, 29.261652e-6
        });

        // Essentially diagonal matrix, so inverse is simple
        invI = CVector3(1.0f / 16.571710e-6, 1.0f / 16.655602e-6, 1.0f / 29.261652e-6);
    }

    CFFirmware::State step(CFFirmware::Action action, float dt, CVector3 fA = CVector3::ZERO)
    {
        float f1 = CFFirmware::rpmToForce(action.M1());
        float f2 = CFFirmware::rpmToForce(action.M2());
        float f3 = CFFirmware::rpmToForce(action.M3());
        float f4 = CFFirmware::rpmToForce(action.M4());

        CMatrix<4,1> mF = CMatrix<4,1>();
        mF(0,0) = f1;
        mF(1,0) = f2;
        mF(2,0) = f3;
        mF(3,0) = f4;

        auto eta = B0*mF;
        CVector3 fU = CVector3(0.0f, 0.0f, eta(0, 0));
        CVector3 tauU = CVector3(eta(0,0), eta(1,0), eta(2, 0));

        CVector3 posNext = state.position + state.velocity * dt;
        CVector3 velNext = state.velocity + (CVector3(0,0,-g) + (((CRotationMatrix3)state.quat * fU) + fA) / mass) * dt;

        CVector3 omegaGlobal = (CRotationMatrix3)state.quat * state.omega;
        auto qNext = IntegrateQuaternion(state.quat, omegaGlobal, dt); // Already normalized in method

        CVector3 omegaRhs = ((CRotationMatrix3)I * state.omega).CrossProduct(state.omega) + tauU;
        CVector3 omegaRhsScaled = CVector3(
            invI.GetX() * omegaRhs.GetX(),
            invI.GetY() * omegaRhs.GetY(),
            invI.GetZ() * omegaRhs.GetZ()
        );

        CVector3 omegaNext = state.omega + omegaRhsScaled * dt;

        state.position = posNext;
        state.velocity = velNext;
        state.quat = qNext;
        state.omega = omegaNext;

        // When falling below the ground, set velocity to zero
        // This disregards ARGoS geometry, but is a workaround for now
        if (state.position.GetZ() < 0.0)
        {
            state.position.SetZ(0.0);
            state.velocity = CVector3::ZERO;
            state.omega = CVector3::ZERO;
        }

        return state;
    }

    
};