#include <potnavi/polar_vector.hpp>
#include <potnavi/math_utils.hpp>
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
* 5 SEARCH     -1      0           0           0
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
    ql::PolarVector fpushVector;
    ql::PolarVector fpullVector;
    State state;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;
    bool isAtGoal = false;
    bool isTargetSeen = false;
    for (auto r : cameraReadings) {
        if (r->Color == CColor::RED || r->Color == CColor::YELLOW || r->Color == CColor::PURPLE) {
            fpullVector += MathUtils::readingToVector(r->Distance, r->Angle, A, B_PULL, C_PULL,
                                                      MathUtils::cameraToDistance);
            isTargetSeen = true;
            if (r->Color == CColor::PURPLE) {
                isAtGoal = true;
            }
        }
    }

    for (int i = 0; i < proxReadings.size(); ++i) {
        if (!MathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += MathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle,
                                                      A, B_PUSH, C_PUSH, MathUtils::proxToDistance);
        }
        if (i == PROX_READING_PER_SIDE) {
            i = proxReadings.size() - 1 - PROX_READING_PER_SIDE - 1;
        }
    }
    fpushVector = -fpushVector;
    fpushVector.clampZeroAndMax(1);
    fpullVector.clampZeroAndMax(1);
    ql::PolarVector directionVector = fpullVector * ALPHA_PULL + fpushVector * BETA_PUSH;
    directionVector.clampZeroAndMax(1);

    mLed->SetAllColors(CColor::WHITE);
    if (isTargetSeen || isAtGoal) {
        mLed->SetAllColors(CColor::YELLOW);
    }
    mPrevState = state;

    std::array<double, 2> wheelSpeeds = {0.0f, 0.0f};

    if (!directionVector.isZero()) {
        wheelSpeeds = ql::MathUtils::vectorToLinearVelocity(directionVector);
    }

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);
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
