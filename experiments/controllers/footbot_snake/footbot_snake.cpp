#include <potnavi/polar_vector.hpp>
#include <potnavi/math_utils.hpp>
#include "footbot_snake.h"

FootbotSnake::FootbotSnake() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        epoch(0) {}

/**
*                           FOLLOW  WANDER STAY AVOID
* 0 NO_TARGET_TO_FOLLOW     0      0.2       -1    0
* 1 TARGET_FOLLOW           0      0       -1    0
* 2 TARGET_REACHED          0      0        2    0
* 3 OBSTACLE_DETECTED       0      0       -1    0
*/
void FootbotSnake::Init(TConfigurationNode &t_node) {

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
    GetNodeAttribute(t_node, "logging", parShouldLog);

    parStage = StageHelper::ParseStageFromString(parStageString);
    srand(12);
    if (parStage == StageHelper::TRAIN) {
        mQLearner = new QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.4);
        std::vector<std::tuple<State, Action>> impossibleStates = {
                std::make_tuple(FollowerState::TARGET_FOLLOW, FollowerAction::STAY),
                std::make_tuple(FollowerState::NO_TARGET_TO_FOLLOW, FollowerAction::STAY),
                std::make_tuple(FollowerState::OBSTACLE_DETECTED, FollowerAction::STAY)
        };
        std::vector<std::tuple<State, Action, double>> rewards = {
                std::make_tuple(FollowerState::TARGET_REACHED, FollowerAction::STAY, 2),
                std::make_tuple(FollowerState::NO_TARGET_TO_FOLLOW, FollowerAction::WANDER, 0.2),
        };
        mQLearner->initR(impossibleStates, rewards, FollowerState::TARGET_REACHED);
        mStateStats.fill(0);
    }
    if (parStage == StageHelper::Stage::EXPLOIT) {
        mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
        mQExploiter->readQ("qmats/snake-train.qlmat");
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
void FootbotSnake::ControlStep() {
    PolarVector pushVector;
    PolarVector pullVector;
    FollowerState state;

    string id = this->m_strId;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;
    bool isAtGoal = false;
    bool isTargetSeen;
    bool isLeaderSeen = false;
    bool isIndirectLeaderSeen = false;
    bool isIndirectLeaderInFrontSeen = false;
    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob closestLeaderBlob(CColor::RED, CRadians::TWO_PI, 1000.0f);
    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob closestIndirectLeaderBlob(CColor::YELLOW, CRadians::TWO_PI, 1000.0f);
    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob closestIndirectLeaderBlobFront(CColor::YELLOW, CRadians::TWO_PI, 1000.0f);
    for (auto r : cameraReadings) {
        if (r->Distance < closestLeaderBlob.Distance && r->Color == CColor::RED) {
            closestLeaderBlob.Angle = r->Angle;
            closestLeaderBlob.Distance = r->Distance;
            isLeaderSeen = true;
        }
        if (r->Distance < closestIndirectLeaderBlob.Distance && r->Color == CColor::YELLOW) {
            closestIndirectLeaderBlob.Angle = r->Angle;
            closestIndirectLeaderBlob.Distance = r->Distance;
            isIndirectLeaderSeen = true;
        } // TODO: PI_OVER_FOUR or PI_OVER_TWO or PI_OVER_SIX?
        if (r->Distance < closestIndirectLeaderBlobFront.Distance && r->Color == CColor::YELLOW && r->Angle.GetAbsoluteValue() < argos::CRadians::PI_OVER_FOUR.GetValue()) {
            closestIndirectLeaderBlobFront.Angle = r->Angle;
            closestIndirectLeaderBlobFront.Distance = r->Distance;
            isIndirectLeaderInFrontSeen = true;
        }
        if (r->Color == CColor::PURPLE) {
            isAtGoal = true;
        }

        if (r->Distance > maxCamera) {
            maxCamera = r->Distance;
        }
    }

    if (isLeaderSeen) {
        pullVector = MathUtils::readingToVector(closestLeaderBlob.Distance, closestLeaderBlob.Angle, A, B_PULL, C_PULL,
                                                MathUtils::cameraToDistance);
    } else if (isIndirectLeaderSeen) {
        if (isIndirectLeaderInFrontSeen) {
            pullVector = MathUtils::readingToVector(closestIndirectLeaderBlobFront.Distance, closestIndirectLeaderBlobFront.Angle, A, B_PULL, C_PULL,
                                                    MathUtils::cameraToDistance);
        } else {
            pullVector = MathUtils::readingToVector(closestIndirectLeaderBlob.Distance, closestIndirectLeaderBlob.Angle, A, B_PULL, C_PULL,
                                                    MathUtils::cameraToDistance);
        }
    }

    isTargetSeen = isIndirectLeaderSeen || isLeaderSeen;

    for (int i = 0; i < proxReadings.size(); ++i) {
        if (!MathUtils::closeToZero(proxReadings.at(i).Value)) {
            pushVector += MathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle,
                                                     A, B_PUSH, C_PUSH, MathUtils::proxToDistance);
        }
        if (i == PROX_READING_PER_SIDE) {
            i = proxReadings.size() - 1 - PROX_READING_PER_SIDE - 1;
        }
    }

    pushVector = -pushVector;
    pushVector.clampZeroAndMax(1);
    pullVector.clamp(0.01, 1);

    bool isTargetFollow = pushVector.isZero() && isTargetSeen;
    bool isTargetReached = isAtGoal && !isTargetSeen;
    bool isObstacleDetected = pushVector.isNotZero() && !isAtGoal && !isTargetSeen;
    bool isObstacleAndTargetDetectedFront = isTargetSeen && pushVector.isNotZero() && pullVector.getAbsAngle() <= 60.0f;
    bool isObstacleAndTargetDetectedBack = isTargetSeen && pushVector.isNotZero() && pullVector.getAbsAngle() > 60.0f; // TODO: Angle?
    bool isNoTargetToFollow = pullVector.isZero() && pushVector.isZero() && !isTargetSeen && !isAtGoal;

    FollowerAction action;
    CColor color = CColor::WHITE;
    if (isNoTargetToFollow) {
        action = FollowerAction::WANDER;
        state = FollowerState::NO_TARGET_TO_FOLLOW;
        color = state.getLedColor();
    } else if (isTargetFollow) {
        state = FollowerState::TARGET_FOLLOW;
        color = state.getLedColor();
        action = FollowerAction::FOLLOW;
        if (!isIndirectLeaderInFrontSeen && !isLeaderSeen || pullVector.isZero()) {
            color = CColor::WHITE;
        }
    } else if (isTargetReached) {
        state = FollowerState::TARGET_REACHED;
        action = FollowerAction::STAY;
        color = state.getLedColor();
    } else if (isObstacleDetected) {
        state = FollowerState::OBSTACLE_DETECTED;
        color = state.getLedColor();
        action = FollowerAction::AVOID;
    } else if (isObstacleAndTargetDetectedFront) {
        state = FollowerState::OBSTACLE_DETECTED_TARGET_FRONT;
        action = FollowerAction::AVOID;
        color = state.getLedColor();
    } else if (isObstacleAndTargetDetectedBack) {
        state = FollowerState::OBSTACLE_DETECTED_TARGET_BACK;
        action = FollowerAction::FOLLOW;
        color = state.getLedColor();
    }

    if (state.getName() == "INVALID") {
        int bp = 0;
    }
    int numStates = isTargetReached + isNoTargetToFollow + isTargetFollow + isObstacleDetected +
            isObstacleAndTargetDetectedBack + isObstacleAndTargetDetectedFront;

    if (numStates > 1) {
        int bp = 0;
    }

    mLed->SetSingleColor(12, color);
    epoch++;
//    if (parStage == StageHelper::Stage::TRAIN) {
//        mStateStats[state.getIndex()] += 1;
//        bool isLearned = true;
//        for (auto a : mStateStats) {
//            if (a < STATE_THRESHOLD) {
//                isLearned = false;
//                break;
//            }
//        }
//        if (isLearned && epoch < mLearnedEpoch) {
//            mQLearner->setLearningRate(0);
//            mLearnedEpoch = epoch;
//        }
//        if (mQLearner->getLearningRate() > 0.05f && epoch % 350 == 0) {
//            mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
//        }
//        LOG << "Learned epoch: " << mLearnedEpoch << std::endl;
//        LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
//
//    }
//    FollowerAction action = (parStage == StageHelper::Stage::EXPLOIT) ?
//                            mQExploiter->exploit<FollowerState, FollowerAction>(state) :
//                            mQLearner->doubleQ<FollowerState, FollowerAction>(mPrevState, state);

    mPrevState = state;

    std::array<double, 2> wheelSpeeds = {0.0f, 0.0f};

    if (action == FollowerAction::FOLLOW) {
        wheelSpeeds = ql::MathUtils::vectorToLinearVelocity(pullVector);
    } else if (action == FollowerAction::AVOID) {
        wheelSpeeds = ql::MathUtils::vectorToLinearVelocity(pushVector);
    } else if (action == FollowerAction::WANDER) {
        wheelSpeeds = {parWheelVelocity, parWheelVelocity};
        if (drand48() > 0.8) {
            if (drand48() > 0.5) {
                wheelSpeeds = {parWheelVelocity, -parWheelVelocity};
            } else {
                wheelSpeeds = {-parWheelVelocity, parWheelVelocity};
            }
        } else {
            wheelSpeeds = {parWheelVelocity, parWheelVelocity};
        }
    } else if (action == FollowerAction::STAY) {
        wheelSpeeds = {0.0, 0.0f};
    }

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);

    LOG << this->m_strId << std::endl;
    LOG << "state: " << state.getName() << std::endl;
    LOG << "action: " << action.getName() << std::endl;
    LOG << "-----------------------------------------------" << std::endl;
}

void FootbotSnake::Destroy() {
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
REGISTER_CONTROLLER(FootbotSnake, "footbot_snake_controller")
