#include <potnavi/polar_vector.hpp>
#include <potnavi/math_utils.hpp>
#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        epoch(0) {}

/**
*                     FOLLOW  WANDER STAY AVOID
* 0 GOAL_REACHED      0      0       1    0
* 1 LEADER_DETECTED   0      0      -1    0
* 2 OBSTACLE_DETECTED 0      0      -1    0
* 3 UNKNOWN           0      0      -1    0
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
    GetNodeAttribute(t_node, "logging", parShouldLog);

    parStage = StageHelper::ParseStageFromString(parStageString);
    if (parStage == StageHelper::TRAIN) {
        mQLearner = new QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.15);
        std::vector<std::tuple<State, Action>> impossibleStates = {
                std::make_tuple(FollowerState::LEADER_DETECTED, FollowerAction::STAY),
                std::make_tuple(FollowerState::OBSTACLE_DETECTED, FollowerAction::STAY),
                std::make_tuple(FollowerState::UNKNOWN, FollowerAction::STAY)
        };
        std::vector<std::tuple<State, Action, double>> rewards = {
                std::make_tuple(FollowerState::GOAL_REACHED, FollowerAction::STAY, 1)
        };
        mQLearner->initR(impossibleStates, rewards, State::IDLE);
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
    PolarVector pushVector;
    PolarVector pullVector;
    PolarVector leaderVector;
    FollowerState state;

    string id = this->m_strId;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;
    bool isAtGoal = false;
    bool isTargetSeen = false;
    bool isLeaderSeen = false;

    PolarVector minVector(20, 0);
    for (auto r : cameraReadings) {
        if (r->Color == CColor::RED || r->Color == CColor::YELLOW || r->Color == CColor::PURPLE) {
            PolarVector targetVector = MathUtils::readingToVector(r->Distance, r->Angle, A, B_PULL, C_PULL,
                                                               MathUtils::cameraToDistance);
            if (targetVector.getLength() < minVector.getLength()) {
                minVector = targetVector;
            }
            pullVector += targetVector;
            isTargetSeen = true;
            if (r->Color == CColor::RED || r->Color == CColor::PURPLE) {
                isLeaderSeen = true;
                leaderVector = targetVector;
            }
//            if (r->Color == CColor::PURPLE) {
//                isAtGoal = true;
//            }
        }
    }

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
    pullVector.clampZeroAndMax(1);

    bool isGoalReached = isLeaderSeen && leaderVector.isZero();
    bool isObstacleDetected = !pushVector.isZero();
    bool isLeaderDetected = isLeaderSeen;
    bool isIndirectLeaderDetected = isTargetSeen;
    bool isUnknown = pushVector.isZero() && pullVector.isZero() && !isTargetSeen;

    FollowerAction action;
    if (mPrevState == FollowerState::LEADER_DETECTED && isObstacleDetected) {

    }
    if (isGoalReached) {
        state = FollowerState::GOAL_REACHED;
        action = FollowerAction::STAY;
    } else if (isObstacleDetected) {
        state = FollowerState::OBSTACLE_DETECTED;
        action = FollowerAction::AVOID;
    } else if (isLeaderDetected) {
        state = FollowerState::LEADER_DETECTED;
        action = FollowerAction::FOLLOW_LEADER;
    } else if (isIndirectLeaderDetected) {
        state = FollowerState::INDIRECT_LEADER_DETECTED;
        action = FollowerAction::FOLLOW_INDIRECT_LEADER;
    } else if (isUnknown) {
        state = FollowerState::UNKNOWN;
        action = FollowerAction::WANDER;
    }
    mLed->SetAllColors(state.getLedColor());

    int numStatesAtATime = isGoalReached + isLeaderDetected + isObstacleDetected + isUnknown;
    if (numStatesAtATime > 1) {
        int bp = 0;
    }

    if (state.getName() == "INVALID") {
        int bp2 = 0;
    }
//    epoch++;
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
//        if (mQLearner->getLearningRate() > 0.05f && epoch % 200 == 0) {
//            mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
//        }
//
//    }

//    Action action = (parStage == StageHelper::Stage::EXPLOIT) ? mQExploiter->exploit(state) : mQLearner->doubleQ(mPrevState, state);


    mPrevState = state;

    std::array<double, 2> wheelSpeeds = {0.0f, 0.0f};

    if (id == "17") {
        int bp = 0;
    }


    if (action == FollowerAction::FOLLOW_LEADER) {
        wheelSpeeds = ql::MathUtils::vectorToLinearVelocity(leaderVector);
    } else if (action == FollowerAction::FOLLOW_INDIRECT_LEADER) {
        wheelSpeeds = ql::MathUtils::vectorToLinearVelocity(pullVector);
    } else if (action == FollowerAction::AVOID) {
        wheelSpeeds = ql::MathUtils::vectorToLinearVelocity(pushVector);
    } else if (action == FollowerAction::WANDER) {
        wheelSpeeds = {parWheelVelocity, parWheelVelocity};
    } else if (action == FollowerAction::STAY) {
        wheelSpeeds = {0.0, 0.0f};
    }

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);

    LOG << this->m_strId << std::endl;
    LOG << "state: " << state.getName() << std::endl;
    LOG << "action: " << action.getName() << std::endl;
    LOG << "-----------------------------------------------" << std::endl;
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
