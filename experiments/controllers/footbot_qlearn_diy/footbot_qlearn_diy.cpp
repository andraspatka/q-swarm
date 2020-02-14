#include "footbot_qlearn_diy.h"

FootbotQLearnDiy::FootbotQLearnDiy() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        globalMaxLightReading(0),
        epoch(0) {}

void FootbotQLearnDiy::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    std::string parStageString;

    GetNodeAttributeOrDefault(t_node, "velocity", parWheelVelocity, parWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "learning_rate", parLearnRate, parLearnRate);
    GetNodeAttributeOrDefault(t_node, "discount_factor", parDiscountFactor, parDiscountFactor);
    GetNodeAttributeOrDefault(t_node, "threshold", parThreshold, parThreshold);
    GetNodeAttributeOrDefault(t_node, "stage", parStageString, parStageString);

    parStage = parseStageFromString(parStageString);
    mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate);
    std::vector<std::tuple<int, int>> impossibleStates = {
            std::make_tuple(0, 0), // WANDER state, STOP action
            std::make_tuple(1, 0), // TURN state, STOP action
            std::make_tuple(2, 0) // AVOID state, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 1), // WANDER state, FORWARD action
            std::make_tuple(3, 0, 1) // IDLE state, STOP action
    };
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT) {
        mQLearner->readQ("Qmat.txt");
    }
}



bool closeToZero(double value) {
    return value < FootbotQLearnDiy::EXP_EPSILON;
}

std::string FootbotQLearnDiy::getActionName(double x, double y) {
    if (x == 0.0 && y == 0.0) return "STOP";
    if (x == parWheelVelocity && y == parWheelVelocity) return "FORWARD";
    if (x == 0.0 && y == parWheelVelocity) return "TURN LEFT";
    if (x == parWheelVelocity && y == 0.0f) return "TURN RIGHT";
    return "INVALID";
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
void FootbotQLearnDiy::ControlStep() {
    double maxFrontLight = 0.0f;
    double maxLight = 0.0f;
    double backMaxLight = 0.0f;
    double turningBackMaxLight = 0.0f;

    double maxProx = 0.0f;
    double backMaxProx = 0.0f;

    double rewardValue = 0;

    int state = 0;

    double action[2];

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

    // back
    for (int i = 10; i <= 13; ++i) {
        backMaxProx = std::max(backMaxProx, proxReadings.at(i).Value);
        backMaxLight = std::max(backMaxLight, lightReadings.at(i).Value);
    }

    for (int i = 5; i <= 8; ++i) {
        turningBackMaxLight = std::max(turningBackMaxLight, lightReadings.at(i).Value);
        turningBackMaxLight = std::max(turningBackMaxLight, lightReadings.at(i + 10).Value);
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
    if (closeToZero(backMaxLight) && closeToZero(maxProx) && closeToZero(backMaxProx) && maxLight < parThreshold) {
         // WANDER state
        state = 0;
    }
    if (backMaxLight > 0.01 && closeToZero(maxProx) && closeToZero(backMaxProx) && maxLight < parThreshold) {
         // TURN state
        state = 1;
    }
    if (maxProx > 0 || backMaxProx > 0) {
         // AVOID state
        state = 2;
    }
    if (maxLight > parThreshold && closeToZero(maxProx)) {
         // IDLE state
         state = 3;
    }

    epoch++;
    if (mQLearner->getLearningRate() > 0.05f && epoch % 100 == 0) { // Every 250 epochs it decreases the parExploreExploit parameter
        mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
    }

    if (globalMaxLightReading < rewardValue) {
        globalMaxLightReading = rewardValue;
    }
    int actionIndex = 0;

    switch (parStage) {
        case Stage::EXPLORE:
            actionIndex = mQLearner->explore(mPrevState, state);
            break;
        case Stage::EXPLOIT:
            actionIndex = mQLearner->exploit(state);
            break;
        case Stage::EXPLORE_EXPLOIT:
            actionIndex = mQLearner->exploreOrExploit(mPrevState, state);
            break;
        case Stage::INVALID:
            actionIndex = 0;
            break;
    }

    globalMaxLightReading = std::max(globalMaxLightReading, maxLight);
    mPrevState = state;
    switch (actionIndex) {
        case 0:
            action[0] = 0;
            action[1] = 0;
            break;
        case 1:
            action[0] = 0;
            action[1] = parWheelVelocity;
            break;
        case 2:
            action[0] = parWheelVelocity;
            action[1] = 0;
            break;
        case 3:
            action[0] = parWheelVelocity;
            action[1] = parWheelVelocity;
            break;
        default:
            action[0] = 0;
            action[1] = 0;
            break;
    }

    mDiffSteering->SetLinearVelocity(action[0], action[1]);

    std::string actualState;
    switch (state) {
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
    LOG << "TurningLight: " << turningBackMaxLight << std::endl;
    LOG << "MaxFrontLight: " << maxFrontLight << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;

    LOG << "Action taken: " << getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Max max light: " << globalMaxLightReading << std::endl;
}

void FootbotQLearnDiy::Destroy() {
    mQLearner->printQ("Qmat.txt");
    delete mQLearner;
}

FootbotQLearnDiy::Stage FootbotQLearnDiy::parseStageFromString(const std::string &stageString) {
    if (stageString == "explore") return FootbotQLearnDiy::Stage::EXPLORE;
    if (stageString == "exploit") return FootbotQLearnDiy::Stage::EXPLOIT;
    if (stageString == "exploreExploit") return FootbotQLearnDiy::Stage::EXPLORE_EXPLOIT;
    std::cerr << "Invalid Stage value: " << stageString;
    exit(1);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearnDiy, "footbot_qlearn_diy_controller")
