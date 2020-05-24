#include <argos3/core/utility/math/vector3.h>
#include <monitoring/logger.hpp>
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

    parStage = StageHelper::ParseStageFromString(parStageString);

    if (parStage == StageHelper::Stage::TRAIN) {
        mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.15);
        std::vector<std::tuple<int, int>> impossibleStates = {
                std::make_tuple(0, 0), // WANDER state, STOP action
                std::make_tuple(1, 0), // FOLLOW state, STOP action
                std::make_tuple(2, 0), // DIR_LEFT state, STOP action
                std::make_tuple(3, 0) // DIR_RIGHT state, STOP action
        };
        std::vector<std::tuple<int, int, double>> rewards = {
                std::make_tuple(0, 3, 0.2), // WANDER state, FORWARD action
                std::make_tuple(1, 3, 1), // FOLLOW state, FORWARD action
                std::make_tuple(4, 0, 2) // IDLE state, STOP action
        };
        mQLearner->initR(impossibleStates, rewards);
    }
    if (parStage == StageHelper::Stage::EXPLOIT) {
        mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
        mQExploiter->readQ("qmats/leader-train.qlmat");
    }

    mLed->SetAllColors(CColor::RED);
    Logger::clearMyLogs(this->m_strId);
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

    CVector2 fpushVector;
    CVector2 fpullVector;

    int state = -1;

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

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

    // max light reading around the footbot
    for (int i = 0; i < 23; ++i) {
        if (!QLMathUtils::closeToZero(lightReadings.at(i).Value)) {
            fpullVector += QLMathUtils::readingToVector(lightReadings.at(i).Value, lightReadings.at(i).Angle,
                    A, B_PULL, C_PULL, QLMathUtils::ligthToDistance);
            maxLight = std::max(maxLight, lightReadings.at(i).Value);
        }
    }

    CVector2 directionVector = fpullVector - fpushVector;

    bool isDirZero = QLMathUtils::closeToZero(directionVector.Length());
    bool isTargetSeen = !QLMathUtils::closeToZero(maxLight);

    // States
    bool isFollow = QLMathUtils::absAngleInDegrees(directionVector.Angle()) <= FORWARD_ANGLE && !isDirZero && maxLight < parThreshold;
    bool isDirLeft = QLMathUtils::angleInDegrees(directionVector.Angle()) > FORWARD_ANGLE &&
                      QLMathUtils::angleInDegrees(directionVector.Angle()) <= SIDE_ANGLE && !isDirZero && maxLight < parThreshold;
    bool isDirRight = QLMathUtils::angleInDegrees(directionVector.Angle()) < -FORWARD_ANGLE &&
                      QLMathUtils::angleInDegrees(directionVector.Angle()) >= -SIDE_ANGLE && !isDirZero && maxLight < parThreshold;
    bool isWander = QLMathUtils::closeToZero(maxProx) && !isTargetSeen;
    bool isIdle = (isDirZero && isTargetSeen) || maxLight > parThreshold;

    std::string actualStateString;
    if (isWander) {
        state = 0;
        actualStateString = "WANDER";
    } else if (isFollow) {
        state = 1;
        actualStateString = "FOLLOW";
    } else if (isDirLeft) {
        state = 2;
        actualStateString = "DIR_LEFT";
    } else if (isDirRight) {
        state = 3;
        actualStateString = "DIR_RIGHT";
    } else if (isIdle) {
        state = 4;
        actualStateString = "IDLE";
    }

    epoch++;
    if (parStage == StageHelper::Stage::TRAIN) {
        mStateStats[state] += 1;
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
        LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    }
    int actionIndex = (parStage == StageHelper::Stage::EXPLOIT) ? mQExploiter->exploit(state) : mQLearner->doubleQ(mPrevState, state);
    mPrevState = state;
    std::array<double, 2> action = QLUtils::getActionFromIndex(actionIndex, parWheelVelocity);
    mGlobalMaxLightReading = std::max(mGlobalMaxLightReading, maxLight);

    mDiffSteering->SetLinearVelocity(action[0], action[1]);
    std::string actionName = QLUtils::getActionName(action[0], action[1]);
    const CVector3 actualPosition = this->mPosition->GetReading().Position;
    std::vector<std::string> toLog = {
            std::to_string(actualPosition.GetX()),
            std::to_string(actualPosition.GetY()),
            actualStateString,
            actionName
    };
    ql::Logger::log(this->m_strId, toLog);

    // LOGGING
    LOG << "Stage: " << StageHelper::ParseStringFromStage(parStage) << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;
    LOG << "Learned epoch: " << mLearnedEpoch << std::endl;

    LOG << "Action taken: " <<  actionName << std::endl;
    LOG << "State: " << actualStateString << std::endl;
    LOG << "Global max light: " << mGlobalMaxLightReading << std::endl;
    LOG << "Id: " << this->m_strId << std::endl;
    LOG << "---------------------------------------------" << std::endl;
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
