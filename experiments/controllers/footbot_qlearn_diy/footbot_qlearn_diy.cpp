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
            std::make_tuple(0, 0), // FOLLOW state, STOP action
            std::make_tuple(1, 0), // UTURN state, STOP action
            std::make_tuple(2, 0), // OBST_LEFT state, STOP action
            std::make_tuple(3, 0), // OBST_RIGHT state, STOP action
            std::make_tuple(4, 0), // OBST_FORWARD state, STOP action
            std::make_tuple(5, 0) // WANDER state, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 1), // FOLLOW state, FORWARD action
            std::make_tuple(2, 2, 0.1), // OBST_LEFT
            std::make_tuple(3, 1, 0.1), // OBST_RIGHT
            std::make_tuple(4, 1, 0.1), // OBST_FORWARD
            std::make_tuple(5, 3, 0.1), // WANDER state, FORWARD action
            std::make_tuple(6, 0, 1) // IDLE state, STOP action
    };
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT || parStage == EXPLORE_EXPLOIT) {
        mQLearner->readQ("Qmat.qlmat");
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
 * Function for aggregating multiple sensor values: max
 * Aggregate of sensor values:
 *      f:  0 23
 *      lm: 5 6
 *      rm: 17 18
 *      lf: 1 2 3 4
 *      rf: 19 20 21 22
 *      b:  8 9 10 11 12 13 14 15
 *
 * States:
 *      FOLLOW - the goal is visible from the front
 *          sensors:
 *              prox: 0
 *              light: b=0
 *      UTURN - facing in the wrong direction, should turn around
 *          sensors:
 *              prox: ?
 *              light: b > 0 && front == 0
 *      OBST_LEFT
 *      OBST_RIGHT
 *      OBST_FORWARD
 *      WANDER
 *      IDLE - end destination reached, GOAL state
 *          sensors:
 *              prox: ?
 *              light: max(sensor values) > THRESHOLD
 */
void FootbotQLearnDiy::ControlStep() {
    double maxLight = 0.0f;
    double backMaxLight = 0.0f;

    double leftMaxProx = 0.0f;
    double rightMaxProx = 0.0f;

    double rewardValue = 0;
    int state = -1;
    double action[2];

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

    // light sensor value from the back of the robot
    for (int i = 8; i <= 15; ++i) {
        backMaxLight = std::max(backMaxLight, lightReadings.at(i).Value);
    }

    // Left front values
    for (int i = 0; i <= 5; ++i) {
        leftMaxProx = std::max(leftMaxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 18; i <= 23; ++i) {
        rightMaxProx = std::max(rightMaxProx, proxReadings.at(i).Value);
    }

    // max light reading around the footbot
    for (int i = 0; i < 23; ++i) {
        maxLight = std::max(maxLight, lightReadings.at(i).Value);
    }

    // States
    if (closeToZero(backMaxLight) && closeToZero(leftMaxProx) && closeToZero(rightMaxProx) && maxLight < parThreshold) {
         // FOLLOW state
        state = 0;
    }
    if (!closeToZero(backMaxLight) && closeToZero(leftMaxProx) && closeToZero(rightMaxProx) && maxLight < parThreshold) {
         // UTURN state
        state = 1;
    }
    if (!closeToZero(leftMaxProx) && closeToZero(rightMaxProx)) {
         // OBST_LEFT state
        state = 2;
    }
    if (!closeToZero(rightMaxProx) && closeToZero(leftMaxProx)) {
        // OBST_RIGHT state
        state = 3;
    }
    if (!closeToZero(leftMaxProx) && !closeToZero(rightMaxProx)) {
        // OBST_FORWARD state
        state = 4;
    }
    if (closeToZero(maxLight) && closeToZero(leftMaxProx) && closeToZero(rightMaxProx)) {
        // WANDER state
        state = 5;
    }
    if (maxLight >= parThreshold) {
         // IDLE state
         state = 6;
    }

    epoch++;
    if (mQLearner->getLearningRate() >= 0.05f && epoch % 150 == 0) {
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
        case 0: // STOP
            action[0] = 0;
            action[1] = 0;
            break;
        case 1: // TURN LEFT
            action[0] = 0;
            action[1] = parWheelVelocity;
            break;
        case 2: // TURN RIGHT
            action[0] = parWheelVelocity;
            action[1] = 0;
            break;
        case 3: // FORWARD
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
            actualState = "FOLLOW";
            break;
        }
        case 1: {
            actualState = "UTURN";
            break;
        }
        case 2: {
            actualState = "OBST_LEFT";
            break;
        }
        case 3: {
            actualState = "OBST_RIGHT";
            break;
        }
        case 4: {
            actualState = "OBST_FORWARD";
            break;
        }
        case 5: {
            actualState = "WANDER";
            break;
        }
        case 6: {
            actualState = "IDLE";
            break;
        }
        default: {
            actualState = "ERROR";
        }
    }
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "LeftMaxProx: " << leftMaxProx << std::endl;
    LOG << "RightMaxProx: " << rightMaxProx << std::endl;
    LOG << "BackMaxLight: " << backMaxLight << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;

    LOG << "Action taken: " << getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Global max light: " << globalMaxLightReading << std::endl;
}

void FootbotQLearnDiy::Destroy() {
    mQLearner->printQ("Qmat.qlmat");
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
