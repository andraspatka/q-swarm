#include "footbot_leader.h"

FootbotLeader::FootbotLeader() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        globalMaxLightReading(0),
        epoch(0) {}

/**
*            STOP    TURN_LEFT   TURN_RIGHT  FORWARD
* 0 WANDER      -1      0           0           0.1
* 1 FOLLOW      -1      0           0           1
* 2 UTURN       -1      0           0           0
* 3 OBST_LEFT   -1      0           0.1         0
* 4 OBST_RIGHT  -1      0.1         0           0
* 5 IDLE         1      0           0           0
*/
void FootbotLeader::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "learning_rate", parLearnRate);
    GetNodeAttribute(t_node, "discount_factor", parDiscountFactor);
    GetNodeAttribute(t_node, "threshold", parThreshold);
    GetNodeAttribute(t_node, "stage", parStageString);

    parStage = parseStageFromString(parStageString);
    mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate);
    std::vector<std::tuple<int, int>> impossibleStates = {
            std::make_tuple(0, 0), // FOLLOW state, STOP action
            std::make_tuple(1, 0), // UTURN state, STOP action
            std::make_tuple(2, 0), // OBST_LEFT state, STOP action
            std::make_tuple(3, 0), // OBST_RIGHT state, STOP action
            std::make_tuple(4, 0) // WANDER state, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 1), // FOLLOW state, FORWARD action
            std::make_tuple(2, 2, 0.1), // OBST_LEFT
            std::make_tuple(3, 1, 0.1), // OBST_RIGHT
            std::make_tuple(4, 3, 0.1), // WANDER state, FORWARD action
            std::make_tuple(5, 0, 1) // IDLE state, STOP action
    };
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT) {
        mQLearner->readQ("qmats/Qmat-train.qlmat");
    }
}

std::string FootbotLeader::getActionName(double x, double y) {
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
 */
void FootbotLeader::ControlStep() {
    double maxLight = 0.0f;
    double backMaxLight = 0.0f;

    double leftMaxProx = 0.0f;
    double rightMaxProx = 0.0f;

    double rewardValue = 0;
    int state = -1;

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

    // light sensor value from the back of the robot
    for (int i = 8; i <= 15; ++i) {
        backMaxLight = std::max(backMaxLight, lightReadings.at(i).Value);
    }

    // Left front values
    for (int i = 0; i <= 4; ++i) {
        leftMaxProx = std::max(leftMaxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 19; i <= 23; ++i) {
        rightMaxProx = std::max(rightMaxProx, proxReadings.at(i).Value);
    }
    mLed->SetAllColors(CColor::RED);
    // max light reading around the footbot
    for (int i = 0; i < 23; ++i) {
        maxLight = std::max(maxLight, lightReadings.at(i).Value);
    }

    // States
    bool isFollow = QLMathUtils::closeToZero(backMaxLight) && QLMathUtils::closeToZero(leftMaxProx) && QLMathUtils::closeToZero(rightMaxProx) &&
                    !QLMathUtils::closeToZero(maxLight) && maxLight < parThreshold;
    bool isUturn = !QLMathUtils::closeToZero(backMaxLight) && QLMathUtils::closeToZero(leftMaxProx) && QLMathUtils::closeToZero(rightMaxProx) &&
                   maxLight < parThreshold;
    bool isObstLeft = leftMaxProx >= rightMaxProx && !QLMathUtils::closeToZero(leftMaxProx) && maxLight < parThreshold;
    bool isObstRight = leftMaxProx < rightMaxProx && maxLight < parThreshold;
    bool isWander = QLMathUtils::closeToZero(maxLight) && QLMathUtils::closeToZero(leftMaxProx) && QLMathUtils::closeToZero(rightMaxProx);
    bool isIdle = maxLight >= parThreshold;

    std::string actualStateString;
    if (isFollow) {
        state = 0;
        actualStateString = "WANDER";
    } else if (isUturn) {
        state = 1;
        actualStateString = "UTURN";
    } else if (isObstLeft) {
        state = 2;
        actualStateString = "OBST_LEFT";
    } else if (isObstRight) {
        state = 3;
        actualStateString = "OBST_RIGHT";
    } else if (isWander) {
        state = 4;
        actualStateString = "WANDER";
    } else if (isIdle) {
        state = 5;
        actualStateString = "IDLE";
    }

    epoch++;
    if (mQLearner->getLearningRate() > 0.05f && epoch % 150 == 0 && parStage == Stage::TRAIN) {
        mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
    }

    if (globalMaxLightReading < rewardValue) {
        globalMaxLightReading = rewardValue;
    }
    int actionIndex = mQLearner->train(mPrevState, state);
    std::array<double, 2> action = QLUtils::getActionFromIndex(actionIndex, parWheelVelocity);
    globalMaxLightReading = std::max(globalMaxLightReading, maxLight);
    mPrevState = state;
    mDiffSteering->SetLinearVelocity(action[0], action[1]);

    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "Stage: " << parseStringFromStage(parStage) << std::endl;
    LOG << "LeftMaxProx: " << leftMaxProx << std::endl;
    LOG << "RightMaxProx: " << rightMaxProx << std::endl;
    LOG << "BackMaxLight: " << backMaxLight << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;

    LOG << "Action taken: " << QLUtils::getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualStateString << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Global max light: " << globalMaxLightReading << std::endl;
    LOG << "Id: " << this->m_strId << std::endl;
}

void FootbotLeader::Destroy() {
    mQLearner->printQ("qmats/Qmat-" + this->m_strId + ".qlmat", false);
    delete mQLearner;
}

FootbotLeader::Stage FootbotLeader::parseStageFromString(const std::string &stageString) {
    if (stageString == "train") return FootbotLeader::Stage::TRAIN;
    if (stageString == "exploit") return FootbotLeader::Stage::EXPLOIT;
    std::cerr << "Invalid Stage value: " << stageString;
    exit(1);
}

std::string FootbotLeader::parseStringFromStage(const FootbotLeader::Stage &stage) {
    if (stage == Stage::TRAIN) return "TRAIN";
    if (stage == Stage::EXPLOIT) return "EXPLOIT";
    std::cerr << "Invalid Stage value: " << stage;
    exit(1);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotLeader, "footbot_leader_controller")
