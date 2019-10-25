#include "footbot_qlearn.h"

SingleFootBot::SingleFootBot() :
        mDiffSteering(NULL),
        mProximity(NULL),
        mWheelVelocity(2.5f),
        mMinDistance(0.25f) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void SingleFootBot::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
}

/**
 * Obstacle avoidance. Inspired from footbot_diffusion.cpp by Carlo Pinciroli
 * The robot avoids obstacles and progresses towards the end goal, marked by the LED light.
 */
void SingleFootBot::ControlStep() {
    CVector2 vectorSum;
    CVector2 straightVector = CVector2(1, 0);

    bool destDetected = false;

    for (auto reading : mProximity->GetReadings()) {
        CVector2 vector = CVector2(reading.Value, reading.Angle);
        // Obstacle detected by the proximity sensor is a pushing force.
        vectorSum += vector;
    }

    // Normalize the vector's length.
    vectorSum /= mProximity->GetReadings().size();

    for (auto reading : mLightSensor->GetReadings()) {
        if (reading.Value != 0) {
            destDetected = true;
        }
        CVector2 vector = CVector2(reading.Value, reading.Angle);
        // Light detected by the light sensor is a pulling force.
        vectorSum -= vector;
    }

    // Normalize the vector's length.
    vectorSum /= mLightSensor->GetReadings().size();

    // If the destination is detected, then the robot should move straight towards it.
    if (destDetected) {
        straightVector = vectorSum;
    }
    // If the dot product of the vectorSum and the "straight" vector is negative
    // in other words: the angle between them is more than 90 degrees.
    // This is needed, so the robot tries to go around the obstacle instead of turning around.
    if ((vectorSum.DotProduct(straightVector) < std::numeric_limits<Real>::epsilon()) && vectorSum.Length() < 0.1f) {
        mLeftWheelVelocity = mWheelVelocity;
        mRightWheelVelocity = mWheelVelocity;
    } else if (vectorSum.Angle().GetValue() < 0) {
        // Angle is negative, turn to the left.
        mLeftWheelVelocity = 0;
        mRightWheelVelocity = mWheelVelocity;
    } else {
        // Angle is positive, turn to the right.
        mLeftWheelVelocity = mWheelVelocity;
        mRightWheelVelocity = 0;
    }

    mDiffSteering->SetLinearVelocity(mLeftWheelVelocity, mRightWheelVelocity);

    LOG << "Vector length: " << vectorSum.Length() << std::endl;
    LOG << "Vector Angle: " << vectorSum.Angle().GetAbsoluteValue() << std::endl;
    LOG << "Vector dot product: " << vectorSum.DotProduct(straightVector) << std::endl;
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(SingleFootBot, "footbot_qlearn_controller")
