#include "footbot_qlearn.h"

FootbotQLearn::FootbotQLearn() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        mWheelVelocity(2.5f),
        mLearner(STATE_DIMENSIONS, minAction, maxAction, BASE_OF_DIMENSIONS) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void FootbotQLearn::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
}

void FootbotQLearn::ControlStep() {
    rl::State states;
    rl::State newStates;
    double maxLightReading = 0.0f;
    int i = 1;
    CVector2 vectorSumGoal(0, 0);
    for (auto reading : mLightSensor->GetReadings()) {
        vectorSumGoal += CVector2(reading.Value, reading.Angle);
        if ( i % 6 == 0  && i != 0) {
            states.push_back(vectorSumGoal.Length());
            vectorSumGoal = CVector2(0, 0);
        }
        ++i;
        if (reading.Value > maxLightReading) {
            maxLightReading = reading.Value;
        }
    }

    rl::Action action = mLearner.chooseBoltzmanAction(states, EPSILON);
    double rewardValue = maxLightReading;
    if (k == 3) {
        mLearner.applyReinforcementToLastAction(rewardValue, states);
        k = 0;
    }
    ++k;

    std::cout<< "Action taken: "<< action[0] << " " << action[1] << std::endl; 
    std::cout<< "Reward: " << rewardValue << std::endl;
    mDiffSteering->SetLinearVelocity(action[0], action[1]);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearn, "footbot_qlearn_controller")
