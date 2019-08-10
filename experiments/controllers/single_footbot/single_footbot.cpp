#include "single_footbot.h"


SingleFootBot::SingleFootBot() :
   mDiffSteering(NULL),
   mProximity(NULL),
   mWheelVelocity(2.5f) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void SingleFootBot::Init(TConfigurationNode& t_node) {
   
   mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   mProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

   GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
}

/**
 * Obstacle avoidance. Inspired from footbot_diffusion.cpp by Carlo Pinciroli
 * Currently the robot avoids the obstacle by turning around.
 *
 * TODO: find best condition for acceptable obstacle distance
 * TODO: define a goal (an end position) and make the footbot progress towards it.
 */
void SingleFootBot::ControlStep() {
    CVector2 vectorSum;

    for (auto reading : mProximity->GetReadings()) {
        CVector2 vector = CVector2(reading.Value, reading.Angle);
        vectorSum += vector;
    }

    // No obstacle detected, or the obstacle is far enough.
    if ((vectorSum.Length() - std::numeric_limits<Real>::epsilon() < 0) || vectorSum.Length() < 0.3f) {
        mLeftWheelVelocity = mWheelVelocity;
        mRightWheelVelocity = mWheelVelocity;
    } else if (vectorSum.Angle().GetValue() < 0) {
        // Angle is negative, turn to the right.
        mLeftWheelVelocity = 0;
        mRightWheelVelocity = mWheelVelocity;
    } else {
        // Angle is positive, turn to the left.
        mLeftWheelVelocity = mWheelVelocity;
        mRightWheelVelocity = 0;
    }

    mDiffSteering->SetLinearVelocity(mLeftWheelVelocity, mRightWheelVelocity);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller
 */
REGISTER_CONTROLLER(SingleFootBot, "single_footbot_controller")
