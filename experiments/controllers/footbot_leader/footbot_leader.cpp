#include <argos3/core/utility/math/vector3.h>
#include <monitoring/logger.hpp>
#include <qlearner/state/simple_state.hpp>
#include <qlearner/action/low_level_action.hpp>
#include "footbot_leader.h"

FootbotLeader::FootbotLeader() : mGlobalMaxLightReading(0){}

/**
*            STOP    TURN_LEFT   TURN_RIGHT  FORWARD
* 0 WANDER     -1      0           0           0.2
* 1 FOLLOW     -1      0           0           1
* 2 DIR_LEFT   -1      0           0           0
* 3 DIR_RIGHT  -1      0           0           0
* 4 IDLE        2      0           0           0
*/
void FootbotLeader::Init(TConfigurationNode &t_node) {
    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    mPosition = GetSensor<CCI_PositioningSensor>("positioning");
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "learning_rate", parLearnRate);
    GetNodeAttribute(t_node, "discount_factor", parDiscountFactor);
    GetNodeAttribute(t_node, "threshold", parThreshold);
    GetNodeAttribute(t_node, "stage", parStageString);
    GetNodeAttribute(t_node, "logging", parShouldLog);
    GetNodeAttribute(t_node, "ignore_goal", parIgnoreGoal);

    parStage = StageHelper::ParseStageFromString(parStageString);

    if (parStage == StageHelper::Stage::TRAIN) {
        mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.15);
        std::vector<std::tuple<State, Action>> impossibleStates = {
                std::make_tuple(SimpleState::WANDER, LowLevelAction::STOP),
                std::make_tuple(SimpleState::FOLLOW, LowLevelAction::STOP),
                std::make_tuple(SimpleState::DIR_LEFT, LowLevelAction::STOP),
                std::make_tuple(SimpleState::DIR_RIGHT, LowLevelAction::STOP)
        };
        std::vector<std::tuple<State, Action, double>> rewards = {
                std::make_tuple(SimpleState::WANDER, LowLevelAction::FORWARD, 0.2),
                std::make_tuple(SimpleState::FOLLOW, LowLevelAction::FORWARD, 1),
                std::make_tuple(SimpleState::IDLE, LowLevelAction::STOP, 2)
        };
        mQLearner->initR(impossibleStates, rewards, SimpleState::IDLE);
    }
    if (parStage == StageHelper::Stage::EXPLOIT) {
        mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
        mQExploiter->readQ("qmats/leader-train.qlmat");
    }

    if (parIgnoreGoal) {
        mLed->SetSingleColor(12, CColor::ORANGE);
    } else {
        mLed->SetSingleColor(12, CColor::RED);
    }
    Logger::clearMyLogs(this->m_strId);
}

void FootbotLeader::Reset() {
    mLed->SetSingleColor(12, CColor::RED);
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
    double maxProx = 0.0f;

    ql::PolarVector fpushVector;
    ql::PolarVector fpullVector;

    SimpleState state;

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

    for (int i = 0; i <= 23; ++i) {
        if (!MathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += MathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle,
                                                      A, B_PUSH, C_PUSH, MathUtils::proxToDistance);
            maxProx = std::max(maxProx, proxReadings.at(i).Value);
        }
        if (i == 4) {
            i = 18;
        }
    }

    // max light reading around the footbot
    if (!parIgnoreGoal) {
        for (int i = 0; i < 23; ++i) {
            if (!MathUtils::closeToZero(lightReadings.at(i).Value)) {
                fpullVector += MathUtils::readingToVector(lightReadings.at(i).Value, lightReadings.at(i).Angle,
                                                          A, B_PULL, C_PULL, MathUtils::lightToDistance);
                maxLight = std::max(maxLight, lightReadings.at(i).Value);
            }
        }
    }

    fpushVector = -fpushVector;
    ql::PolarVector directionVector = fpullVector + fpushVector;
    directionVector.clampZeroAndMax(1);

    bool isDirZero = MathUtils::closeToZero(directionVector.getLength());
    bool isTargetSeen = !MathUtils::closeToZero(maxLight);
    bool isAtGoal = maxLight > parThreshold;

    // States
    bool isFollow = directionVector.getAbsAngle() <= FORWARD_ANGLE && !isDirZero && !isAtGoal;
    bool isDirLeft = directionVector.getAngle() > FORWARD_ANGLE &&
                      directionVector.getAngle() <= SIDE_ANGLE && !isDirZero && !isAtGoal;
    bool isDirRight = directionVector.getAngle() < -FORWARD_ANGLE &&
                      directionVector.getAngle() > -SIDE_ANGLE && !isDirZero && !isAtGoal;
    bool isWander = MathUtils::closeToZero(maxProx) && !isTargetSeen;
    bool isIdle = (isDirZero && isTargetSeen) || maxLight > parThreshold;

    if (isWander) {
        state = SimpleState::WANDER;
    } else if (isFollow) {
        state = SimpleState::FOLLOW;
    } else if (isDirLeft) {
        state = SimpleState::DIR_LEFT;
    } else if (isDirRight) {
        state = SimpleState::DIR_RIGHT;
    } else if (isIdle) {
        state = SimpleState::IDLE;
        mLed->SetSingleColor(12, CColor::PURPLE);
    }

    epoch++;
    if (parStage == StageHelper::Stage::TRAIN) {
        mStateStats[state.getIndex()] += 1;
        bool isLearned = true;
        for (auto a : mStateStats) {
            if (a < STATE_THRESHOLD) {
                isLearned = false;
            }
        }
        if (isLearned && epoch < mLearnedEpoch) {
            mQLearner->setLearningRate(0);
            mLearnedEpoch = epoch;
        }
        if (mQLearner->getLearningRate() > 0.05f && epoch % 200 == 0) {
            mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
        }
    }
    LowLevelAction action = (parStage == StageHelper::Stage::EXPLOIT) ? mQExploiter->exploit<SimpleState, LowLevelAction>(state) :
            mQLearner->doubleQ<SimpleState, LowLevelAction>(mPrevState, state);
    mPrevState = state;
    std::array<double, 2> wheelSpeeds = {0, 0};
    if (action == LowLevelAction::FORWARD) {
        wheelSpeeds = {1.0, 1.0};
    } else if (action == LowLevelAction::TURN_LEFT) {
        wheelSpeeds = {-1.0, 1.0};
    } else if (action == LowLevelAction::TURN_RIGHT) {
        wheelSpeeds = {1.0, -1.0};
    } else if (action == LowLevelAction::STOP) {
        wheelSpeeds = {0.0, 0.0};
    }

    wheelSpeeds[0] = wheelSpeeds[0] * parWheelVelocity;
    wheelSpeeds[1] = wheelSpeeds[1] * parWheelVelocity;

    mGlobalMaxLightReading = std::max(mGlobalMaxLightReading, maxLight);

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);

    if (parShouldLog) {
        auto position = this->mPosition->GetReading().Position;
        ql::Logger::logPositionStateAndAction(position.GetX(), position.GetY(), state.getName(), action.getName(), this->m_strId);

        if (parStage == StageHelper::Stage::TRAIN) {
            LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
        }
        LOG << "Stage: " << StageHelper::ParseStringFromStage(parStage) << std::endl;
        LOG << "MaxLight: " << maxLight << std::endl;
        LOG << "Learned epoch: " << mLearnedEpoch << std::endl;

        LOG << "Action taken: " << action.getName() << std::endl;
        LOG << "State: " << state.getName() << std::endl;
        LOG << "Global max light: " << mGlobalMaxLightReading << std::endl;
        LOG << "Id: " << this->m_strId << std::endl;
        LOG << "---------------------------------------------" << std::endl;
    }
}

void FootbotLeader::Destroy() {
    if (parStage == StageHelper::Stage::TRAIN) {
        mQLearner->printQ("qmats/" + this->m_strId + ".qlmat", true);
        delete mQLearner;
    }
    if (parStage == StageHelper::Stage::EXPLOIT) {
        delete mQExploiter;
    }
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotLeader, "footbot_leader_controller")
