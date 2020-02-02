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
    GetNodeAttributeOrDefault(t_node, "threshold", mThreshold, mThreshold);
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
    double discountFactor = 0.7f; // 0 - immediate reward, 1 - long term reward

    mWireFitQLearner = new rl::WireFitQLearn(
            STATE_DIMENSIONS, ACTION_LENGTH, numHiddenLayers, numNeuronsPerHiddenLayer, numberOfWires, minAction,
            maxAction, BASE_OF_DIMENSIONS, mLSInterpolator, mTrainer, wireFitQLearningRate, discountFactor);
}

bool closeToZero(double value) {
    return value < FootbotQLearn::EXP_EPSILON;
}

/**
 *              front
 *
 *              0 23
 *            1     22
 *          2         21
 *        3             20      r
 * l    4                 19    i
 * e  5                     18  g
 * f  6                     17  h
 * t    7                 16    t
 *        8             15
 *          9         14
 *            10     13
 *              11 12
 *
 *              back
 * Aggregate of sensor values:
 *      f:  0 23
 *      lm: 5 6
 *      rm: 17 18
 *      lf: 1 2 3 4
 *      rf: 19 20 21 22
 *      b:  7 8 9 10 11 12 13 14 15 16
 *
 * States:
 *      WANDER - explorational phase
 *          reward: 0.1
 *          sensors:
 *              prox: 0
 *              light: b=0
 *      IDLE - end destination reached
 *          reward: 1
 *          sensors:
 *              prox: ?
 *              light: max(sensor values) > THRESHOLD
 *      TURN - facing in the wrong direction
 *          reward: -0.2
 *          sensors:
 *              prox: ?
 *              light: b > 0 && front == 0
 *      AVOID - obstacle detected
 *          reward: -1
 *          sensors:
 *              prox: != 0
 *              light: ?
 */
void FootbotQLearn::ControlStep() {
    rl::State states;
    double maxFrontLight = 0.0f;
    double maxLight = 0.0f;
    double backMaxLight = 0.0f;

    double maxProx = 0.0f;
    double backMaxProx = 0.0f;

    double rewardValue = 0;

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

    // back
    for (int i = 10; i <= 13; ++i) {
        backMaxLight = std::max(backMaxLight, lightReadings.at(i).Value);
        backMaxProx = std::max(backMaxProx, proxReadings.at(i).Value);
    }

    // Left front values
    for (int i = 0; i <= 4; ++i) {
        maxFrontLight = std::max(maxFrontLight, lightReadings.at(i).Value);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 19; i <= 23; ++i) {
        maxFrontLight = std::max(maxFrontLight, lightReadings.at(i).Value);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }
    for (int i = 0; i < 23; ++i) {
        maxLight = std::max(maxLight, lightReadings.at(i).Value);
    }

    // States
    if (closeToZero(backMaxLight) && closeToZero(maxProx)) {
        states.push_back(1); // WANDER state
        rewardValue = 0.1f;
    } else {
        states.push_back(0);
    }
    if (backMaxLight > 0.01) {
        states.push_back(1); // TURN state
        rewardValue = -backMaxLight;
    } else {
        states.push_back(0);
    }
    if (maxProx > 0 || backMaxProx > 0) {
        states.push_back(1); // AVOID state
        rewardValue = -2 * maxProx - 2 * backMaxProx;
    } else {
        states.push_back(0);
    }
    if (maxLight > mThreshold) { // Threshold value is initialized from .argos file
        states.push_back(1); // IDLE state
        rewardValue = 1;
    } else {
        states.push_back(0);
    }

    epoch++;
    if (exploreExploit > 0.4f && epoch % 250 == 0) { // Every 250 epochs it decreases the exploreExploit parameter
        exploreExploit -= 0.1f;
    }
    rl::Action action = mWireFitQLearner->chooseBoltzmanAction(states, exploreExploit);
    mWireFitQLearner->applyReinforcementToLastAction(rewardValue, states);

    if (maxReward < rewardValue) {
        maxReward = rewardValue;
    }
    mDiffSteering->SetLinearVelocity(action[0], action[1]);

    int statei = 0;
    for (int i = 0; i < states.size(); ++i) {
        if (states.at(i) == 1) {
            statei = i;
        }
    }
    std::string actualState;
    switch (statei) {
        case 0: {
            actualState = "WANDER";
            break;
        }
        case 1: {
            actualState = "TURN";
            break;
        }
        case 2: {
            actualState = "AVOID";
            break;
        }
        case 3: {
            actualState = "IDLE";
            break;
        }
        default: {
            actualState = "ERROR";
        }
    }
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "BackwProx: " << backMaxProx << std::endl;
    LOG << "MaxProx: " << maxProx << std::endl;
    LOG << "BackwLight: " << backMaxLight << std::endl;
    LOG << "MaxFrontLight: " << maxFrontLight << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;

    LOG << "Action taken: " << action[0] << " " << action[1] << std::endl;
    LOG << "Reward: " << rewardValue << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "ExploreExploit: " << exploreExploit << std::endl;
    LOG << "Max reward: " << maxReward << std::endl;
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
