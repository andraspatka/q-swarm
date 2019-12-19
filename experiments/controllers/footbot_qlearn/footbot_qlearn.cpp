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
    double distanceToGoal = 0;
    double distanceToObstacle = 0;
    int i = 1;
    CVector2 vectorSumGoal, vectorSumObstacle;
    for (auto reading : mLightSensor->GetReadings()) {
        vectorSumGoal += CVector2(reading.Value, reading.Angle);
        if ( i % 6 == 0  && i != 0) {
            states.push_back(vectorSumGoal.Length());
            vectorSumGoal = CVector2(0, 0);
        }
        ++i;
        if (reading.Value > distanceToGoal) {
            distanceToGoal = reading.Value;
        }
    }
    /*for (auto reading : mProximitySensor->GetReadings()) {
        //states.push_back(reading.Value);
        vectorSumObstacle += CVector2(reading.Value, reading.Angle);
        if (reading.Value > distanceToGoal) {
            distanceToObstacle = reading.Value;
        }
    }*/

    rl::Action action = mLearner.chooseBoltzmanAction(states, EPSILON);
    //CVector2 actionVector(action[0], action[1]);
    //CVector2 newPositionGoal = actionVector + vectorSumGoal;
    //CVector2 newPositionObstacle = actionVector + vectorSumObstacle;
    //TODO: calculate the reward
    double rewardValue = -distanceToGoal;
    if (k == 5) {
        mLearner.applyReinforcementToLastAction(rewardValue, states);
        states = newStates;
        k = 0;
    }
    ++k;


    mDiffSteering->SetLinearVelocity(action[0], action[1]);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearn, "footbot_qlearn_controller")
