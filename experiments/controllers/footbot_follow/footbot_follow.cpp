#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        globalMinCameraBlobDist(0),
        epoch(0) {}

/**
 *
 *            STOP    TURN_LEFT   TURN_RIGHT  FORWARD
 * 0 WANDER     -1      0           0           0.2
 * 1 FOLLOW     -1      0           0           1
 * 2 DIR_LEFT   -1      0           0           0
 * 3 DIR_RIGHT  -1      0           0           0
 * 4 IDLE        2      0           0           0
 */
void FootbotFollow::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    mPosition = GetSensor<CCI_PositioningSensor>("positioning");

    mCamera->Enable();
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "learning_rate", parLearnRate);
    GetNodeAttribute(t_node, "discount_factor", parDiscountFactor);
    GetNodeAttribute(t_node, "threshold", parThreshold);
    GetNodeAttribute(t_node, "stage", parStageString);
    GetNodeAttribute(t_node, "qmat_filename", parQMatFileName);

    parStage = parseStageFromString(parStageString);
    mQLearner = new QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.1);
    std::vector<std::tuple<int, int>> impossibleStates = {
            std::make_tuple(0, 0), // WANDER state, STOP action
            std::make_tuple(1, 0), // FOLLOW state, STOP action
            std::make_tuple(2, 0), // DIR_LEFT, STOP action
            std::make_tuple(3, 0)  // DIR_RIGHT, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 0.2), // WANDER state, FORWARD action
            std::make_tuple(1, 3, 1), // FOLLOW state, FORWARD action
            std::make_tuple(4, 0, 2), // IDLE state, STOP action
    };
    mStateStats.fill(0);
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT) {
        mQLearner->readQ("qmats/Qfollow-train.qlmat");
    }
    ql::Logger::clearMyLogs(this->m_strId);
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
 */
void FootbotFollow::ControlStep() {
    CVector2 fpushVector;
    CVector2 fpullVector;

    double maxProx = 0.0f;

    int state = -1;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;

    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob minDistanceBlob(CColor::WHITE, CRadians::TWO_PI, 1000.0f);
    for (auto r : cameraReadings) {
        if (r->Distance < minDistanceBlob.Distance && (r->Color == CColor::RED || r->Color == CColor::YELLOW)) {
            minDistanceBlob.Distance = r->Distance;
            minDistanceBlob.Angle = r->Angle;
            minDistanceBlob.Color = r->Color;

            fpullVector += QLMathUtils::readingToVector(r->Distance, r->Angle, A, B_PULL, C_PULL,
                                                            QLMathUtils::cameraToDistance);
        }
    }

    // Left front values
    for (int i = 0; i <= 4; ++i) {
        fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                        C_PUSH, QLMathUtils::proxToDistance);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 19; i <= 23; ++i) {
        fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                        C_PUSH, QLMathUtils::proxToDistance);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }

    CVector2 directionVector = fpullVector - fpushVector;
    bool isDirZero = QLMathUtils::closeToZero(directionVector.Length());

    double const FORWARD_ANGLE = 30.0f;
    double const SIDE_ANGLE = 180.0f;

    bool isTargetSeen = minDistanceBlob.Distance != 1000.0f;

    bool isWander = QLMathUtils::closeToZero(maxProx) && !isTargetSeen;
    bool isFollow = QLMathUtils::absAngleInDegrees(directionVector.Angle()) < FORWARD_ANGLE && !isDirZero;
    bool isDirLeft = QLMathUtils::angleInDegrees(directionVector.Angle()) > FORWARD_ANGLE &&
                     QLMathUtils::angleInDegrees(directionVector.Angle()) <= SIDE_ANGLE && !isDirZero;
    bool isDirRight = QLMathUtils::angleInDegrees(directionVector.Angle()) < -FORWARD_ANGLE &&
                      QLMathUtils::angleInDegrees(directionVector.Angle()) > -SIDE_ANGLE && !isDirZero;
    bool isIdle = isDirZero && isTargetSeen;

    std::string actualState;
    // States
    if (isWander) {
        actualState = "WANDER";
        state = 0;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isFollow) {
        actualState = "FOLLOW";
        state = 1;
        mLed->SetAllColors(CColor::YELLOW);
    } else if (isDirLeft) {
        actualState = "DIR_LEFT";
        state = 2;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isDirRight) {
        actualState = "DIR_RIGHT";
        state = 3;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isIdle) {
        actualState = "IDLE";
        state = 4;
        mLed->SetAllColors(CColor::GREEN);
    }

    epoch++;
    if (parStage == Stage::TRAIN) {
        mStateStats[state] += 1;
    }
    bool isLearned = true;
    for (auto a : mStateStats) {
        if (a < STATE_THRESHOLD) {
            isLearned = false;
        }
    }
    if (isLearned && epoch < mLearnedEpoch && parStage == Stage::TRAIN) {
        mQLearner->setLearningRate(0);
        mLearnedEpoch = epoch;
        mQLearner->printQ("qmats/debug" + parQMatFileName  + "-" + this->m_strId + ".qlmat", true);
    }
    if (mQLearner->getLearningRate() > 0.05f && epoch % 115 == 0 && parStage == Stage::TRAIN) {
        mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
    }

    int actionIndex = (parStage == Stage::EXPLOIT) ? mQLearner->exploit(state) : mQLearner->doubleQ(mPrevState, state);

    mPrevState = state;

    std::array<double, 2> action = QLUtils::getActionFromIndex(actionIndex, parWheelVelocity);
    mDiffSteering->SetLinearVelocity(action[0], action[1]);

    const CVector3 actualPosition = this->mPosition->GetReading().Position;
    ql::Logger::logPosition(this->m_strId, {actualPosition.GetX(), actualPosition.GetY()});
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "Id: " << this->m_strId << std::endl;
    LOG << "Stage: " << parStage << std::endl;
    LOG << "fpush: " << fpushVector << std::endl;
    LOG << "fpull: " << fpullVector << std::endl;
    LOG << "Direction: " << directionVector << std::endl;
    LOG << "Learned epoch: " << mLearnedEpoch << std::endl;

    LOG << "Action taken: " << QLUtils::getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Global min camera: " << globalMinCameraBlobDist << std::endl;
}

void FootbotFollow::Destroy() {
    if (parStage != Stage::EXPLOIT) {
        mQLearner->printQ("qmats/" + parQMatFileName  + "-" + this->m_strId + ".qlmat", true);
    }

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
