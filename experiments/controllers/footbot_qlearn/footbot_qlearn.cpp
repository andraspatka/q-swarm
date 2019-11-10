#include "footbot_qlearn.h"

FootbotQLearn::FootbotQLearn() :
        mDiffSteering(NULL),
        mProximity(NULL),
        mWheelVelocity(2.5f),
        mMinDistance(0.25f),
        mLearner(STATE_DIMENSIONS, minAction, maxAction, BASE_OF_DIMENSIONS) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void FootbotQLearn::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
}

void FootbotQLearn::ControlStep() {
    rl::State states;
    double distance = 0;
    for (auto reading : mLightSensor->GetReadings()) {
        states.push_back(reading.Value);
        if (reading.Value > distance) {
            distance = reading.Value;
        }
    }

    rl::Action action = mLearner.chooseBoltzmanAction(states, EPSILON);

    /* TODO: calculate the reward
    sf::Vector2f displacement =  sf::Vector2f(action[0], action[1]);
    sf::Vector2f newPosition = simulator.robot.getPosition() + displacement;
    simulator.robot.setPosition(newPosition);

    double newDistance = simulator.distanceFromLine();
    double rewardValue = (fabs(distance) - fabs(newDistance - distance)) / sqrt(2);
    mLearner.applyReinforcementToLastAction(-distance, states);*/

    mDiffSteering->SetLinearVelocity(action[0], action[1]);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearn, "footbot_qlearn_controller")
