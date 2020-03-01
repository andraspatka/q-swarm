#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        globalMaxCameraReading(0),
        epoch(0) {}

/**
 *
 *            STOP    TURN_LEFT   TURN_RIGHT  FORWARD
 * FOLLOW     -1      0           0           1
 * WANDER     -1      0           0           0.1
 * UTURN      -1      0           0           0
 * OBST_LEFT  -1      0           0.1         0
 * OBST_RIGHT -1      0.1         0           0
 * IDLE        1      0           0           0
 */
void FootbotFollow::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    mCamera->Enable();
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "learning_rate", parLearnRate);
    GetNodeAttribute(t_node, "discount_factor", parDiscountFactor);
    GetNodeAttribute(t_node, "threshold", parThreshold);
    GetNodeAttribute(t_node, "stage", parStageString);
    GetNodeAttribute(t_node, "qmat_filename", parQMatFileName);

    parStage = parseStageFromString(parStageString);
    mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate);
    std::vector<std::tuple<int, int>> impossibleStates = {
            std::make_tuple(0, 0), // FOLLOW state, STOP action
            std::make_tuple(1, 0), // WANDER state, STOP action
            std::make_tuple(2, 0), // UTURN, STOP action
            std::make_tuple(3, 0), // OBST_LEFT, STOP action
            std::make_tuple(4, 0)  // OBST_RIGHT, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 1), // FOLLOW state, FORWARD action
            std::make_tuple(1, 3, 0.1), // WANDER state, FORWARD action
            std::make_tuple(3, 2, 0.1), // OBST_LEFT state, TURN_RIGHT action
            std::make_tuple(4, 1, 0.1), // OBST_RIGHT state, TURN_LEFT action
            std::make_tuple(5, 0, 1), // IDLE state, STOP action
    };
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT) {
        mQLearner->readQ("qmats/" + parQMatFileName);
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
 */
void FootbotFollow::ControlStep() {

    double leftMaxProx = 0.0f;
    double rightMaxProx = 0.0f;

    double rewardValue = 0;
    int state = -1;
    double action[2];

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;

    bool leaderDetected = false;
    bool backLeaderDetected = false;
    double maxCameraLen = 0;
    for (auto r : cameraReadings) {
        LOG << r->Color << " " << r->Angle << " " << r->Distance << std::endl;
        double angleInDegrees = r->Angle.GetAbsoluteValue() * argos::CRadians::RADIANS_TO_DEGREES;
        if (angleInDegrees <= 30.0f && (r->Color == CColor::RED || r->Color == CColor::YELLOW || r->Color == CColor::GREEN)) {
            leaderDetected = true;
            maxCameraLen = std::max(maxCameraLen, r->Distance);
        } else if (r->Color == CColor::RED || r->Color == CColor::YELLOW || r->Color == CColor::GREEN) {
            backLeaderDetected = true;
            maxCameraLen = std::max(maxCameraLen, r->Distance);
        }
    }

    // Left front values
    for (int i = 0; i <= 5; ++i) {
        leftMaxProx = std::max(leftMaxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 18; i <= 23; ++i) {
        rightMaxProx = std::max(rightMaxProx, proxReadings.at(i).Value);
    }

    // As we use the camera sensor, the maxCameraLen refers to distance, not sensor reading intensity.
    // Therefore the threshold that we define (the threshold refers to how close the agent should be to its goal) should be
    // less then the maximal camera distance in the given step

    // States
    if (leaderDetected && maxCameraLen > parThreshold) {
         // FOLLOW state
        state = 0;
        mLed->SetAllColors(CColor::YELLOW);
    }
    if (closeToZero(maxCameraLen) && closeToZero(leftMaxProx) && closeToZero(rightMaxProx)) {
        // WANDER state
        state = 1;
        mLed->SetAllColors(CColor::WHITE);
    }
    if (backLeaderDetected && !leaderDetected && closeToZero(leftMaxProx) && closeToZero(rightMaxProx)) {
        // UTURN state
        state = 2;
        mLed->SetAllColors(CColor::WHITE);
    }
    if (leftMaxProx >= rightMaxProx && !closeToZero(leftMaxProx)) {
        // OBST_LEFT state
        state = 3;
        mLed->SetAllColors(CColor::WHITE);
    }
    if (leftMaxProx < rightMaxProx) {
        // OBST_RIGHT state
        state = 4;
        mLed->SetAllColors(CColor::WHITE);
    }
    if (maxCameraLen < parThreshold && (leaderDetected || backLeaderDetected)) {
         // IDLE state
         state = 5;
         mLed->SetAllColors(CColor::GREEN);
    }

    epoch++;
    if (mQLearner->getLearningRate() > 0.05f && epoch % 150 == 0 && parStage == Stage::TRAIN) {
        mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
    }

    if (globalMaxCameraReading < rewardValue) {
        globalMaxCameraReading = rewardValue;
    }
    int actionIndex = mQLearner->train(mPrevState, state);

    globalMaxCameraReading = std::max(globalMaxCameraReading, maxCameraLen);
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
            actualState = "WANDER";
            break;
        }
        case 2: {
            actualState = "UTURN";
            break;
        }
        case 3: {
            actualState = "OBST_LEFT";
            break;
        }
        case 4: {
            actualState = "OBST_RIGHT";
            break;
        }
        case 5: {
            actualState = "IDLE";
            break;
        }
        default: {
            actualState = "ERROR";
        }
    }
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "Stage: " << parStage << std::endl;
    LOG << "LeftMaxProx: " << leftMaxProx << std::endl;
    LOG << "RightMaxProx: " << rightMaxProx << std::endl;
    LOG << "Max camera len: " << maxCameraLen << std::endl;

    LOG << "Action taken: " << getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Global max camera: " << globalMaxCameraReading << std::endl;
}

void FootbotFollow::Destroy() {
    mQLearner->printQ("qmats/" + parQMatFileName);
    delete mQLearner;
}

FootbotFollow::Stage FootbotFollow::parseStageFromString(const std::string &stageString) {
    if (stageString == "train") return FootbotFollow::Stage::TRAIN;
    if (stageString == "exploit") return FootbotFollow::Stage::EXPLOIT;
    std::cerr << "Invalid Stage value: " << stageString;
    exit(1);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotFollow, "footbot_follow")
