#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        globalMaxLightReading(0),
        epoch(0) {}

/**
 *
 *          STOP    TURN_LEFT   TURN_RIGHT  FORWARD
 * FOLLOW   -1      0           0           1
 * AVOID    -1      0           0           0
 * WANDER   -1      0           0           0
 * IDLE     1       0           0           0
 */
void FootbotFollow::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    mCamera->Enable();
    std::string parStageString;

    GetNodeAttributeOrDefault(t_node, "velocity", parWheelVelocity, parWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "learning_rate", parLearnRate, parLearnRate);
    GetNodeAttributeOrDefault(t_node, "discount_factor", parDiscountFactor, parDiscountFactor);
    GetNodeAttributeOrDefault(t_node, "threshold", parThreshold, parThreshold);
    GetNodeAttributeOrDefault(t_node, "stage", parStageString, parStageString);
    GetNodeAttributeOrDefault(t_node, "qmat_filename", parQMatFileName, parQMatFileName);

    parStage = parseStageFromString(parStageString);
    mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate);
    std::vector<std::tuple<int, int>> impossibleStates = {
            std::make_tuple(0, 0), // FOLLOW state, STOP action
            std::make_tuple(1, 0), // AVOID state, STOP action
            std::make_tuple(2, 0) // WANDER state, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 1), // FOLLOW state, FORWARD action
            std::make_tuple(3, 0, 1) // IDLE state, STOP action
    };
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT) {
        mQLearner->readQ(parQMatFileName);
    }
}



bool closeToZero(double value) {
    return value < FootbotFollow::EXP_EPSILON;
}

std::string FootbotFollow::getActionName(double x, double y) {
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
 *      AVOID - obstacle detected
 *          sensors:
 *              prox: front != 0
 *              light: ?
 *      IDLE - end destination reached, GOAL state
 *          sensors:
 *              prox: ?
 *              light: max(sensor values) > THRESHOLD
 *      WANDER - nothing is visible
 *          camera = 0
 *          prox != 0
 */
void FootbotFollow::ControlStep() {
    double frontMaxLight = 0.0f;
    double maxLight = 0.0f;
    double backMaxLight = 0.0f;

    double frontMaxProx = 0.0f;

    double rewardValue = 0;
    int state = 0;
    double action[2];

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;

    bool leaderDetected = false;
    double maxCameraLen = 0;
    for (auto r : cameraReadings) {
        LOG << r->Color << " " << r->Angle << " " << r->Distance;
        if (r->Angle.GetAbsoluteValue() * argos::CRadians::RADIANS_TO_DEGREES > 30.0f) {
            leaderDetected = true;
        }
        std::max(maxCameraLen, r->Distance);
    }

    // Left front values
    for (int i = 0; i <= 4; ++i) {
        frontMaxProx = std::max(frontMaxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 19; i <= 23; ++i) {
        frontMaxProx = std::max(frontMaxProx, proxReadings.at(i).Value);
    }

    // States
    if (leaderDetected && maxCameraLen < parThreshold) {
         // FOLLOW state
        state = 0;
    }
    if (frontMaxProx > 0 && maxCameraLen < parThreshold) {
        // AVOID state
        state = 1;
    }
    if (closeToZero(maxCameraLen) && closeToZero(frontMaxProx)) {
        // WANDER state
        state = 2;
    }
    if (maxLight > parThreshold && leaderDetected) {
         // IDLE state
         state = 3;
    }

    epoch++;
    if (mQLearner->getLearningRate() > 0.05f && epoch % 150 == 0) {
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
            actualState = "AVOID";
            break;
        }
        case 2: {
            actualState = "WANDER";
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
    LOG << "FrontMaxProx: " << frontMaxProx << std::endl;
    LOG << "BackMaxLight: " << backMaxLight << std::endl;
    LOG << "MaxFrontLight: " << frontMaxLight << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;

    LOG << "Action taken: " << getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Global max light: " << globalMaxLightReading << std::endl;
}

void FootbotFollow::Destroy() {
    mQLearner->printQ(parQMatFileName);
    delete mQLearner;
}

FootbotFollow::Stage FootbotFollow::parseStageFromString(const std::string &stageString) {
    if (stageString == "explore") return FootbotFollow::Stage::EXPLORE;
    if (stageString == "exploit") return FootbotFollow::Stage::EXPLOIT;
    if (stageString == "exploreExploit") return FootbotFollow::Stage::EXPLORE_EXPLOIT;
    std::cerr << "Invalid Stage value: " << stageString;
    exit(1);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotFollow, "footbot_follow")
