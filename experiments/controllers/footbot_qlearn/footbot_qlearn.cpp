#include "footbot_qlearn.h"

FootbotQLearn::FootbotQLearn() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        mWheelVelocity(30.0f) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void FootbotQLearn::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
//    GetNodeAttributeOrDefault(t_node, "explore_exploit", exploreExploit, exploreExploit);
    initWireFitQLearn();
}

void FootbotQLearn::initWireFitQLearn() {
    // The action that the agent can take is moving, using its differential steering actuator.
    rl::Action minAction = {0.0f, 0.0f};
    rl::Action maxAction = {mWheelVelocity, mWheelVelocity};

    // How many values the state vector can take.
    const int STATE_DIMENSIONS = 8;
    // number of possible discrete values between minAction and maxAction
    const int BASE_OF_DIMENSIONS = 2;
    const int ACTION_LENGTH = 2;

    // Initialize the backpropagation NN trainer
    double learningRate = 0.05f;
    double momentumTerm = 0.9f; // between 0 and 1, rec: 0.9 tunable: 0.8 - 0.999
    double targetErrorLevel = 0.001f;
    int maxEpochs = 5000;
    // Todo: Use Adadelta instead? => Adaptive learning rate
    mTrainer = new net::Backpropagation(learningRate, momentumTerm, targetErrorLevel, maxEpochs);
    double rho = 0.25f;
    //mTrainer = new net::Adadelta(rho, targetErrorLevel, maxEpochs);

    // Initialize the Least Square Interpolator
    double smoothingFactor = 0.85;
    double e = 0.001f;
    mLSInterpolator = new rl::LSInterpolator(smoothingFactor, e);

    unsigned int numHiddenLayers = 2; // 2 hidden layers => can learn an arbitrary decision boundary
    // 8 neurons input, 4 neurons output => 2/3 inputN + outputN = 9 (rule of thumb, Jeff Heaton)
    unsigned int numNeuronsPerHiddenLayer = 9;
    unsigned int numberOfWires = 4;
    double wireFitQLearningRate = 0.6f; // between 0 and 1 - how fast the agent should learn from the reinforcement
    double discountFactor = 0.6f; // 0 - immediate reward, 1 - long term reward

    mWireFitQLearner = new rl::WireFitQLearn(
                    STATE_DIMENSIONS, ACTION_LENGTH, numHiddenLayers, numNeuronsPerHiddenLayer, numberOfWires, minAction,
                    maxAction, BASE_OF_DIMENSIONS, mLSInterpolator, mTrainer, wireFitQLearningRate, discountFactor);
}

void FootbotQLearn::ControlStep() {
    rl::State states;
    double maxLightReading = 0.0f;
    double rewardValue = 0;

//  *              front
//  *
//  *              0 23
//  *            1     22
//  *          2         21
//  *        3             20      r
//  * l    4                 19    i
//  * e  5                     18  g
//  * f  6                     17  h
//  * t    7                 16    t
//  *        8             15
//  *          9         14
//  *            10     13
//  *              11 12
//  *
//  *              back

    CVector2 vectorSumGoal(0, 0);
    auto readings = mLightSensor->GetReadings();
    // forward
    vectorSumGoal = CVector2(readings.at(0).Value, readings.at(0).Angle) + CVector2(readings.at(23).Value, readings.at(23).Angle);
    states.push_back(vectorSumGoal.Length());
    // left 1
    vectorSumGoal = CVector2(readings.at(1).Value, readings.at(1).Angle) + CVector2(readings.at(2).Value, readings.at(2).Angle);
    states.push_back(vectorSumGoal.Length());
    // left 2
    vectorSumGoal = CVector2(readings.at(3).Value, readings.at(3).Angle) + CVector2(readings.at(4).Value, readings.at(4).Angle);
    states.push_back(vectorSumGoal.Length());
    // leftmost
    vectorSumGoal = CVector2(readings.at(5).Value, readings.at(5).Angle) + CVector2(readings.at(6).Value, readings.at(6).Angle);
    states.push_back(vectorSumGoal.Length());

    // right 1
    vectorSumGoal = CVector2(readings.at(21).Value, readings.at(21).Angle) + CVector2(readings.at(22).Value, readings.at(22).Angle);
    states.push_back(vectorSumGoal.Length());

    // right 2
    vectorSumGoal = CVector2(readings.at(19).Value, readings.at(19).Angle) + CVector2(readings.at(20).Value, readings.at(20).Angle);
    states.push_back(vectorSumGoal.Length());

    // rightmost
    vectorSumGoal = CVector2(readings.at(17).Value, readings.at(17).Angle) + CVector2(readings.at(18).Value, readings.at(18).Angle);
    states.push_back(vectorSumGoal.Length());

    // back
    vectorSumGoal = CVector2(0,0);
    for (int i=7; i <= 16; ++i) {
        vectorSumGoal += CVector2(readings.at(i).Value, readings.at(i).Angle);    
    }
    states.push_back(vectorSumGoal.Length());

    for (int i = 0; i <= 3; ++i) {
        if (readings.at(i).Value > maxLightReading) {
             maxLightReading = readings.at(i).Value;
         }
    }
    for (int i = 20; i <= 23; ++i) {
        if (readings.at(i).Value > maxLightReading) {
             maxLightReading = readings.at(i).Value;
         }
    }
    epoch++;
    if (exploreExploit > 0.4f && epoch % 250 == 0) { // Every 500 epochs it decreases the exploreExploit parameter
        exploreExploit -= 0.1f;
    }
    rl::Action action = mWireFitQLearner->chooseBoltzmanAction(states, exploreExploit);
    rewardValue = maxLightReading;
    if (maxReward < rewardValue) {
        maxReward = rewardValue;
    }
    mWireFitQLearner->applyReinforcementToLastAction(rewardValue, states);

    /*if (action[0] == 0.0f && action[1] == 0.0f) { //{0, 0} action does nothing.
        action[0] = mWheelVelocity;
        action[1] = mWheelVelocity;
    }*/
    LOG<< "Action taken: "<< action[0] << " " << action[1] << std::endl;
    LOG<< "Reward: " << rewardValue << std::endl;
    LOG<< "ExploreExploit: " << exploreExploit << std::endl;
    LOG<< "Max reward: " << maxReward << std::endl;
    mDiffSteering->SetLinearVelocity(action[0], action[1]);
}

void FootbotQLearn::Destroy() {
    delete mWireFitQLearner;
    delete mTrainer;
    delete mLSInterpolator;
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearn, "footbot_qlearn_controller")
