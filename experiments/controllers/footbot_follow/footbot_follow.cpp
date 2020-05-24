#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        mGlobalMinCameraBlobDist(0),
        epoch(0) {}

/**
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

    parStage = StageHelper::ParseStageFromString(parStageString);
    if (parStage == StageHelper::TRAIN) {
        mQLearner = new QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.15);
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
        mQLearner->initR(impossibleStates, rewards);
        mStateStats.fill(0);
    }
    if (parStage == StageHelper::Stage::EXPLOIT) {
        mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
        mQExploiter->readQ("qmats/follow-train.qlmat");
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

            CVector2 pullVector = QLMathUtils::readingToVector(r->Distance, r->Angle, A, B_PULL, C_PULL,
                                                               QLMathUtils::cameraToDistance);
            if (r->Color == CColor::RED) { // following the actual leader has a higher priority
                pullVector = pullVector * 1.4;
            }
            fpullVector = fpullVector + pullVector;
        }
    }

    for (int i = 0; i <= 23; ++i) {
        if (!QLMathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle,
                                                        A, B_PUSH, C_PUSH, QLMathUtils::proxToDistance);
            maxProx = std::max(maxProx, proxReadings.at(i).Value);
        }
        if (i == 4) {
            i = 18;
        }
    }

    CVector2 directionVector = fpullVector - fpushVector;
    bool isDirZero = QLMathUtils::closeToZero(directionVector.Length());

    bool isTargetSeen = minDistanceBlob.Distance != 1000.0f;

    bool isWander = QLMathUtils::closeToZero(maxProx) && !isTargetSeen;
    bool isFollow = QLMathUtils::absAngleInDegrees(directionVector.Angle()) < FORWARD_ANGLE && !isDirZero;
    bool isDirLeft = QLMathUtils::angleInDegrees(directionVector.Angle()) > FORWARD_ANGLE &&
                     QLMathUtils::angleInDegrees(directionVector.Angle()) <= SIDE_ANGLE && !isDirZero;
    bool isDirRight = QLMathUtils::angleInDegrees(directionVector.Angle()) < -FORWARD_ANGLE &&
                      QLMathUtils::angleInDegrees(directionVector.Angle()) >= -SIDE_ANGLE && !isDirZero;
    bool isIdle = isDirZero && isTargetSeen;

    bool negateVelocity = true;
    std::string actualState;
    // States
    if (isWander) {
        actualState = "WANDER";
        state = 0;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isFollow) {
        actualState = "FOLLOW";
        state = 1;
        negateVelocity = false;
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
    if (parStage == StageHelper::Stage::TRAIN) {
        mStateStats[state] += 1;
        bool isLearned = true;
        for (auto a : mStateStats) {
            if (a < STATE_THRESHOLD) {
                isLearned = false;
                break;
            }
        }
        if (isLearned && epoch < mLearnedEpoch) {
            mQLearner->setLearningRate(0);
            mLearnedEpoch = epoch;
        }
        if (mQLearner->getLearningRate() > 0.05f && epoch % 200 == 0) {
            mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
        }
        LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    }
    int actionIndex = (parStage == StageHelper::Stage::EXPLOIT) ? mQExploiter->exploit(state) : mQLearner->doubleQ(mPrevState, state);

    mPrevState = state;

    double velocityFactor = (negateVelocity) ? 1 : directionVector.Length();
    std::array<double, 2> action = QLUtils::getActionFromIndex(actionIndex, parWheelVelocity);
    std::string actionName = QLUtils::getActionName(action[0], action[1]);
    mDiffSteering->SetLinearVelocity(action[0] * velocityFactor, action[1] * velocityFactor);

    const CVector3 actualPosition = this->mPosition->GetReading().Position;
    std::vector<std::string> toLog = {
            std::to_string(actualPosition.GetX()),
            std::to_string(actualPosition.GetY()),
            actualState,
            actionName
    };
    ql::Logger::log(this->m_strId, toLog);

    // LOGGING
    LOG << "Id: " << this->m_strId << std::endl;
    LOG << "Stage: " << parStage << std::endl;
    LOG << "fpush: " << fpushVector << std::endl;
    LOG << "fpull: " << fpullVector << std::endl;
    LOG << "Direction: " << directionVector << std::endl;
    LOG << "VelocityFactor: " << velocityFactor << std::endl;
    LOG << "Learned epoch: " << mLearnedEpoch << std::endl;
    LOG << "Action taken: " << actionName << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "---------------------------------------------" << std::endl;
}

void FootbotFollow::Destroy() {
    if (parStage == StageHelper::Stage::TRAIN) {
        mQLearner->printQ("qmats/" + this->m_strId + ".qlmat", true);
        delete mQLearner;
    }
    if (parStage == StageHelper::EXPLOIT) {
        delete mQExploiter;
    }
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotFollow, "footbot_follow_controller")
