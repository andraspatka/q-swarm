#include <potnavi/polar_vector.hpp>
#include <potnavi/math_utils.hpp>
#include <qlearner/state/simple_state.hpp>
#include <qlearner/action/low_level_action.hpp>
#include "footbot_infect_rw.h"

InfectRandomWalk::InfectRandomWalk() :
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
void InfectRandomWalk::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");
    mPosition = GetSensor<CCI_PositioningSensor>("positioning");

    mCamera->Enable();
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "infectious", parNoOfInfectious);
    GetNodeAttribute(t_node, "infect_prob", parInfectionProb);
    GetNodeAttribute(t_node, "sick_for", parSickFor);
    GetNodeAttribute(t_node, "mortality", parMortality);
    GetNodeAttribute(t_node, "social_distancing", parShouldSocialDistance);
    GetNodeAttribute(t_node, "stage", parStageString);
    GetNodeAttribute(t_node, "asymptomatic", parAsymptomaticRate);

    parStage = StageHelper::ParseStageFromString(parStageString);

    if (parShouldSocialDistance) {
        GetNodeAttribute(t_node, "social_distance_conform", parSocialDistancingConformity);
    }
    srand48(time(nullptr));

    if (parStage == StageHelper::Stage::TRAIN) {
        srand48(5);
        GetNodeAttribute(t_node, "learning_rate", parLearnRate);
        GetNodeAttribute(t_node, "discount_factor", parDiscountFactor);
        mQLearner = new ql::QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate, 0.2);
        std::vector<std::tuple<State, Action>> impossibleStates = {
                std::make_tuple(SimpleState::WANDER, LowLevelAction::STOP),
                std::make_tuple(SimpleState::FOLLOW, LowLevelAction::STOP),
                std::make_tuple(SimpleState::DIR_LEFT, LowLevelAction::STOP),
                std::make_tuple(SimpleState::DIR_RIGHT, LowLevelAction::STOP)
        };
        std::vector<std::tuple<State, Action, double>> rewards = {
                std::make_tuple(SimpleState::WANDER, LowLevelAction::FORWARD, 1),
                std::make_tuple(SimpleState::FOLLOW, LowLevelAction::FORWARD, 1),
                std::make_tuple(SimpleState::IDLE, LowLevelAction::STOP, 2)
        };
        mQLearner->initR(impossibleStates, rewards, SimpleState::IDLE);
    }
    if (parStage == StageHelper::Stage::EXPLOIT) {
        mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
        mQExploiter->readQ("qmats/covid-train.qlmat");
    }
    mId = this->GetId();
    InitInfectious();
}

void InfectRandomWalk::Reset() {
    InitInfectious();
}

void InfectRandomWalk::InitInfectious() {
    ql::Logger::clearMyLogs(mId, true);
    if (parStage == StageHelper::Stage::TRAIN) {
        agentType = AgentTypeHelper::AgentType::SUSCEPTIBLE;
        return;
    }
    int idNumber = std::stoi(mId);
    mInfectedForEpochs = 0;
    if (idNumber < parNoOfInfectious) {
        agentType = AgentTypeHelper::AgentType::INFECTIOUS;
    } else {
        agentType = AgentTypeHelper::AgentType::SUSCEPTIBLE;
    }

    if (parShouldSocialDistance) {
        mConformsToSocialDistancing = idNumber < parSocialDistancingConformity;
    }
    if (drand48() < parAsymptomaticRate) {
        mIsAsymptomatic = true;
    }
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
void InfectRandomWalk::ControlStep() {
    ql::PolarVector fpushVector;
    ql::PolarVector fpullVector;

    PolarVector socialDistancingVector;

    double maxLight = 0.0f;
    SimpleState state;
    epoch++;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;
    auto lightReadings = mLightSensor->GetReadings();

    bool infectedSeesInfectious = false;
    short countInfectious = 0;
    for (auto r : cameraReadings) {
        if (ql::MathUtils::cameraToDistance(r->Distance) <= 1.2 && r->Color == CColor::ORANGE && agentType == AgentTypeHelper::AgentType::SUSCEPTIBLE) {
            countInfectious++;
        }
        socialDistancingVector += MathUtils::readingToVector(r->Distance, r->Angle,
                                                             A, B_PUSH_CAMERA, C_PUSH_CAMERA, MathUtils::cameraToDistance);
        if (r->Color == CColor::ORANGE && agentType == AgentTypeHelper::AgentType::INFECTIOUS) {
            infectedSeesInfectious = true;
        }
    }
    mPrevMaxInfectiousSeen = std::max(countInfectious, mPrevMaxInfectiousSeen);
    if (mPrevMaxInfectiousSeen > 0 && countInfectious == 0) {
        for (int i = 0; i < mPrevMaxInfectiousSeen; ++i) {
            if (drand48() < parInfectionProb) {
                agentType = AgentTypeHelper::AgentType::INFECTIOUS;
                break;
            }
        }
        mPrevMaxInfectiousSeen = 0;
    }


    for (auto r : lightReadings) {
        if (!MathUtils::closeToZero(r.Value)) {
            if (agentType == AgentTypeHelper::AgentType::INFECTIOUS && !mIsAsymptomatic) {
                fpullVector += MathUtils::readingToVector(r.Value, r.Angle,
                                                          A, B_PULL, C_PULL, MathUtils::lightToDistance);
            } else if (agentType != AgentTypeHelper::AgentType::DECEASED) {
                fpushVector += MathUtils::readingToVector(r.Value, r.Angle,
                                                          A, 0, C_PUSH_CAMERA, MathUtils::lightToDistance);
            }
            maxLight = std::max(maxLight, r.Value);
        }
    }

    for (int i = 0; i <= 23; ++i) {
        if (!MathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += MathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                      C_PUSH, MathUtils::proxToDistance);
        }
    }

    if (mConformsToSocialDistancing) {
        fpushVector += socialDistancingVector;
    }


    ql::PolarVector directionVector = - fpushVector;
    directionVector += fpullVector;
    bool isDirZero = directionVector.isZero();

    bool isDeceased = agentType == AgentTypeHelper::DECEASED;
    bool isInIsolation = maxLight > LIGHT_READING_THRESHOLD && agentType == AgentTypeHelper::AgentType::INFECTIOUS ||
            maxLight > EXTENDED_LIGHT_READING_THRESHOLD && infectedSeesInfectious;

    bool isIdle = isInIsolation || isDeceased;
    bool isWander = isDirZero && !isIdle;
    bool isFollow = directionVector.getAbsAngle() <= FORWARD_ANGLE && !isDirZero && !isIdle;
    bool isDirLeft = directionVector.getAngle() > FORWARD_ANGLE &&
                     directionVector.getAngle() <= SIDE_ANGLE && !isDirZero && !isIdle;
    bool isDirRight = directionVector.getAngle() < -FORWARD_ANGLE &&
                      directionVector.getAngle() >= -SIDE_ANGLE && !isDirZero && !isIdle;

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
    }

    if (mInfectedForEpochs == 0 && agentType == AgentTypeHelper::AgentType::INFECTIOUS) {
        mDiesAfterEpochs = rand() % 250 + 51;
        if (drand48() < parMortality) {
            mIsGoingToDie = true;
        }
    }

    if (mInfectedForEpochs > parSickFor) {
        agentType = AgentTypeHelper::AgentType::RECOVERED;
    }
    if (agentType == AgentTypeHelper::AgentType::INFECTIOUS && mInfectedForEpochs > mDiesAfterEpochs && mIsGoingToDie) {
        agentType = AgentTypeHelper::AgentType::DECEASED;
    }

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
        LOG << "Learned epoch: " << mLearnedEpoch << std::endl;
        LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    }

    LowLevelAction action = (parStage == StageHelper::Stage::EXPLOIT) ? mQExploiter->exploit<SimpleState, LowLevelAction>(state) :
                    mQLearner->doubleQ<SimpleState, LowLevelAction>(mPrevState, state);

    mPrevState = state;

    switch (agentType) {
        case AgentTypeHelper::AgentType::INFECTIOUS:
            mInfectedForEpochs++;
            if (!parShouldSocialDistance) {
                mCamera->Disable();
            }
            mLed->SetAllColors(CColor::RED);
            mLed->SetSingleColor(12, CColor::ORANGE);
            break;
        case AgentTypeHelper::AgentType::SUSCEPTIBLE:
            mLed->SetAllColors(CColor::CYAN);
            break;
        case AgentTypeHelper::AgentType::DECEASED:
            mLed->SetAllColors(CColor::GRAY50);
            mCamera->Disable();
            break;
        case AgentTypeHelper::AgentType::RECOVERED:
            mLed->SetAllColors(CColor::GREEN);
            if (!parShouldSocialDistance) {
                mCamera->Disable();
            }
            break;
    }

    if (state == SimpleState::WANDER && parStage == StageHelper::Stage::EXPLOIT) {
        if (drand48() > 0.85f) {
            if (drand48() < 0.5f) {
                action = LowLevelAction::TURN_RIGHT;
            } else {
                action = LowLevelAction::TURN_LEFT;
            }
        } else {
            action = LowLevelAction::FORWARD;
        }
    }
    std::array<double, 2> wheelSpeeds = {0, 0};
    if (action == LowLevelAction::FORWARD) {
        wheelSpeeds = {1.0, 1.0};
    } else if (action == LowLevelAction::TURN_LEFT) {
        wheelSpeeds = {0, 1.0};
    } else if (action == LowLevelAction::TURN_RIGHT) {
        wheelSpeeds = {1.0, 0};
    } else if (action == LowLevelAction::STOP) {
        wheelSpeeds = {0.0, 0.0};
    }

    wheelSpeeds[0] *= parWheelVelocity;
    wheelSpeeds[1] *= parWheelVelocity;

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);

    const CVector3 actualPosition = this->mPosition->GetReading().Position;
    std::vector<std::string> toLog = {
            std::to_string(actualPosition.GetX()),
            std::to_string(actualPosition.GetY()),
            state.getName(),
            action.getName(),
            AgentTypeHelper::GetAgentTypeAsString(this->agentType)
    };
    ql::Logger::log(mId, toLog, true);

    LOG << "Id: " << mId << std::endl;
    LOG << "Type: " << AgentTypeHelper::GetAgentTypeAsString(this->agentType) << std::endl;
    LOG << "State: " << state.getName() << std::endl;
    LOG << "Action: " << action.getName() << std::endl;
    LOG << "max light: " << maxLight << std::endl;
    LOG << "---------------------------------------------" << std::endl;
}

void InfectRandomWalk::Destroy() {
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
REGISTER_CONTROLLER(InfectRandomWalk, "infect_random_walk")
