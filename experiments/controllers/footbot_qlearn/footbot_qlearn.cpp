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
    const int STATE_DIMENSIONS = 4;
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
    double wireFitQLearningRate = 0.7f; // between 0 and 1 - how fast the agent should learn from the reinforcement
    double discountFactor = 0.9f; // 0 - immediate reward, 1 - long term reward

    mWireFitQLearner = new rl::WireFitQLearn(
                    STATE_DIMENSIONS, ACTION_LENGTH, numHiddenLayers, numNeuronsPerHiddenLayer, numberOfWires, minAction,
                    maxAction, BASE_OF_DIMENSIONS, mLSInterpolator, mTrainer, wireFitQLearningRate, discountFactor);
}

void FootbotQLearn::ControlStep() {
    rl::State states;
    double maxLightReading = 0.0f;
    double maxProxReading = 0.0f;
    double backwardsProxReading = 0.0f;
    double backwardsLightReading = 0.0f;
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

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();
    // back
    CVector2 vectorSumGoal = CVector2(0, 0);
    CVector2 vectorSumObst = CVector2(0, 0);
    for (int i = 9; i <= 14; ++i) {
        vectorSumGoal += CVector2(lightReadings.at(i).Value, lightReadings.at(i).Angle);    
        vectorSumObst += CVector2(proxReadings.at(i).Value, proxReadings.at(i).Angle);    
    }
    backwardsLightReading = vectorSumGoal.Length();
    backwardsProxReading = vectorSumObst.Length();

    for (int i = 0; i <= 3; ++i) {
        if (lightReadings.at(i).Value > maxLightReading) {
             maxLightReading = lightReadings.at(i).Value;
         }
         if (proxReadings.at(i).Value > maxProxReading) {
             maxProxReading = proxReadings.at(i).Value;
         }
    }
    for (int i = 20; i <= 23; ++i) {
        if (lightReadings.at(i).Value > maxLightReading) {
             maxLightReading = lightReadings.at(i).Value;
        }
        if (proxReadings.at(i).Value > maxProxReading) {
             maxProxReading = proxReadings.at(i).Value;
        }
    }

    // States
    if (backwardsLightReading < 0.1f && maxProxReading == 0 && backwardsProxReading == 0) {
        states.push_back(1); // WANDER state
        rewardValue = 0.1f;
    } else {
        states.push_back(0);
    }
    if (backwardsLightReading > 0 && maxProxReading == 0 && backwardsProxReading == 0) {
        states.push_back(1); // BACK state
        rewardValue = -0.3f;
    } else {
        states.push_back(0);
    }
    if (maxProxReading > 0) {
        states.push_back(1); // OBSTACLE_DETECTED state
        rewardValue = -1;
    } else {
        states.push_back(0);
    }
    if (maxLightReading > 0.25f) {
        states.push_back(1); // GOAL state
        rewardValue = 1;
    } else {
        states.push_back(0);
    }


    epoch++;
    if (exploreExploit > 0.4f && epoch % 250 == 0) { // Every 250 epochs it decreases the exploreExploit parameter
        exploreExploit -= 0.1f;
    }
    rl::Action action = mWireFitQLearner->chooseBoltzmanAction(states, exploreExploit);
    
    if (maxReward < rewardValue) {
        maxReward = rewardValue;
    }
    mWireFitQLearner->applyReinforcementToLastAction(rewardValue, states);

    /*if (action[0] == 0.0f && action[1] == 0.0f) { //{0, 0} action does nothing.
        action[0] = mWheelVelocity;
        action[1] = mWheelVelocity;
    }*/
    LOG<< "BackwProx: " << backwardsProxReading << std::endl; 
    LOG<< "MaxProx: " << maxProxReading << std::endl;
    LOG<< "BackwLight: " << backwardsLightReading << std::endl;
    LOG<< "MaxLight: " << maxLightReading << std::endl;

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
