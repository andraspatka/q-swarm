#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
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
        std::vector<std::tuple<State, Action>> impossibleStates = {
                std::make_tuple(State::WANDER, Action::STOP),
                std::make_tuple(State::FOLLOW, Action::STOP),
                std::make_tuple(State::DIR_LEFT, Action::STOP),
                std::make_tuple(State::DIR_RIGHT, Action::STOP)
        };
        std::vector<std::tuple<State, Action, double>> rewards = {
                std::make_tuple(State::WANDER, Action::FORWARD, 0.2),
                std::make_tuple(State::FOLLOW, Action::FORWARD, 1),
                std::make_tuple(State::IDLE, Action::STOP, 2)
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
    ql::Vector fpushVector;
    ql::Vector fpullVector;
    State state;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;
    bool isAtGoal = false;
    bool isTargetSeen = false;
    for (auto r : cameraReadings) {
        if (r->Color == CColor::PURPLE) {
            isAtGoal = true;
        }
        if (r->Color == CColor::RED || r->Color == CColor::YELLOW || r->Color == CColor::PURPLE) {
            CVector2 pullVector = QLMathUtils::readingToVector(r->Distance, r->Angle, A, B_PULL, C_PULL,
                                                               QLMathUtils::cameraToDistance);

            if (r->Color != CColor::YELLOW) {
                pullVector *= 1.4;
            }
            fpullVector += pullVector;

            isTargetSeen = true;
        }
    }

    for (int i = 0; i < proxReadings.size(); ++i) {
        if (!QLMathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle,
                                                        A, B_PUSH, C_PUSH, QLMathUtils::proxToDistance);
        }
        if (i == PROX_READING_PER_SIDE) {
            i = proxReadings.size() - 1 - PROX_READING_PER_SIDE - 1;
        }
    }
    ql::Vector directionVector = ALPHA_PULL * fpullVector - BETA_PUSH * fpushVector;
    directionVector.clampLength(0.001, 1);
    bool isDirZero = directionVector.isZero();

    bool isWander = isDirZero && !isTargetSeen; // TODO: isAtGoal is a HACK
    bool isFollow = directionVector.getAbsAngle() < FORWARD_ANGLE && !isDirZero && !isAtGoal;
    bool isDirLeft = directionVector.getAngle() > FORWARD_ANGLE &&
                     directionVector.getAngle() <= SIDE_ANGLE && !isDirZero && !isAtGoal;
    bool isDirRight = directionVector.getAngle() < -FORWARD_ANGLE &&
                      directionVector.getAngle() >= -SIDE_ANGLE && !isDirZero && !isAtGoal;
    bool isIdle = isDirZero && isTargetSeen || isAtGoal;

    CColor ledColor = CColor::WHITE;
    bool negateVelocity = false;
    if (isWander) {
        state = State::WANDER;
        ledColor = state.getLedColor();
        negateVelocity = true;
    } else if (isFollow) {
        state = State::FOLLOW;
        if (isTargetSeen) {
            ledColor = CColor::YELLOW;
        } else {
            ledColor = CColor::WHITE;
        }
    } else if (isDirLeft) {
        state = State::DIR_LEFT;
        ledColor = state.getLedColor();
    } else if (isDirRight) {
        state = State::DIR_RIGHT;
        ledColor = state.getLedColor();
    } else if (isIdle) {
        state = State::IDLE;
        if (isAtGoal) {
            ledColor = CColor::YELLOW;
        } else {
            ledColor = CColor::GREEN;
        }
    }
    mLed->SetAllColors(ledColor);
    epoch++;
    if (parStage == StageHelper::Stage::TRAIN) {
        mStateStats[state.getIndex()] += 1;
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
    Action action = (parStage == StageHelper::Stage::EXPLOIT) ? mQExploiter->exploit(state) : mQLearner->doubleQ(mPrevState, state);

    mPrevState = state;

    double velocityFactor = (negateVelocity) ? 1 : directionVector.Length();
    std::array<double, 2> wheelSpeeds = action.getWheelSpeed();

    wheelSpeeds[0] = wheelSpeeds[0] * parWheelVelocity * velocityFactor;
    wheelSpeeds[1] = wheelSpeeds[1] * parWheelVelocity * velocityFactor;

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);

    const CVector3 actualPosition = this->mPosition->GetReading().Position;
    std::vector<std::string> toLog = {
            std::to_string(actualPosition.GetX()),
            std::to_string(actualPosition.GetY()),
            state.getStateName(),
            action.getActionName()
    };
    ql::Logger::log(this->m_strId, toLog);

    // LOGGING
    LOG << "Id: " << this->m_strId << std::endl;
    LOG << "Stage: " << parStage << std::endl;
    LOG << "fpush: " << fpushVector << std::endl;
    LOG << "fpull: " << fpullVector << std::endl;
    LOG << "Direction: " << directionVector.Length() << std::endl;
    LOG << "VelocityFactor: " << velocityFactor << std::endl;
    LOG << "Learned epoch: " << mLearnedEpoch << std::endl;
    LOG << "Action taken: " << action.getActionName() << std::endl;
    LOG << "State: " << state.getStateName() << std::endl;
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
