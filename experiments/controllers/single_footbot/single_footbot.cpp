#include "single_footbot.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>

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
 * Currently the robot's logic consists of only going forward.
 * All the time.
 */
void SingleFootBot::ControlStep() {
   mDiffSteering->SetLinearVelocity(mWheelVelocity, mWheelVelocity);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller
 */
REGISTER_CONTROLLER(SingleFootBot, "single_footbot_controller")
