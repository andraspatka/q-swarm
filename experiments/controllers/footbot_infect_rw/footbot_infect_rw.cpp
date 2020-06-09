#include <potnavi/polar_vector.hpp>
#include <potnavi/math_utils.hpp>
#include "footbot_infect_rw.h"

InfectRandomWalk::InfectRandomWalk() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        epoch(0) {}

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

    if (parShouldSocialDistance) {
        GetNodeAttribute(t_node, "social_distance_conform", parSocialDistancingConformity);
    }


    mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
    mQExploiter->readQ("qmats/flock-train.qlmat");
    mId = this->GetId();
    InitInfectious();
}

void InfectRandomWalk::Reset() {
    InitInfectious();
}

void InfectRandomWalk::InitInfectious() {
    ql::Logger::clearMyLogs(mId);
    int idNumber = std::stoi(mId);
    mInfectedForEpochs = 0;
    if (idNumber < parNoOfInfectious) {
        agentType = AgentTypeHelper::AgentType::INFECTIOUS;
    } else {
        agentType = AgentTypeHelper::AgentType::SUSCEPTIBLE;
    }

    mDiesAfterEpochs = rand() % 250 + 51;
    if (drand48() < parMortality) {
        isGoingToDie = true;
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

    double maxLight = 0.0f;
    State state;
    epoch++;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;
    auto lightReadings = mLightSensor->GetReadings();

    short countInfectious = 0;
    for (auto r : cameraReadings) {
        if (ql::MathUtils::cameraToDistance(r->Distance) <= 1.2 && r->Color == CColor::ORANGE && agentType == AgentTypeHelper::AgentType::SUSCEPTIBLE) {
            countInfectious++;
        }
    }
    mPrevMaxInfectiousSeen = std::max(countInfectious, mPrevMaxInfectiousSeen);
    if (mPrevMaxInfectiousSeen > 0 && countInfectious == 0) {
        LOG << "Prev count infectious" << mPrevMaxInfectiousSeen << std::endl;
        for (int i = 0; i < mPrevMaxInfectiousSeen; ++i) {
            if (drand48() < parInfectionProb) {
                agentType = AgentTypeHelper::AgentType::INFECTIOUS;
            }
        }
        mPrevMaxInfectiousSeen = 0;
    }

    if (agentType == AgentTypeHelper::AgentType::INFECTIOUS) {
        for (auto r : lightReadings) {
            if (!MathUtils::closeToZero(r.Value)) {
                fpullVector += MathUtils::readingToVector(r.Value, r.Angle,
                                                          A, B_PULL, C_PULL, MathUtils::lightToDistance);
                maxLight = std::max(maxLight, r.Value);
            }
        }
    }

    for (int i = 0; i <= 23; ++i) {
        if (!MathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += MathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                      C_PUSH, MathUtils::proxToDistance);
        }
    }

    ql::PolarVector directionVector = - fpushVector;
    directionVector += fpullVector;
    bool isDirZero = directionVector.isZero();

    bool isDoneWithDisease = agentType == AgentTypeHelper::DECEASED;

    bool isIdle = maxLight > LIGHT_READING_THRESHOLD || isDoneWithDisease;
    bool isWander = isDirZero && !isIdle;
    bool isFollow = directionVector.getAbsAngle() <= FORWARD_ANGLE && !isDirZero && !isIdle;
    bool isDirLeft = directionVector.getAngle() > FORWARD_ANGLE &&
                     directionVector.getAngle() <= SIDE_ANGLE && !isDirZero && !isIdle;
    bool isDirRight = directionVector.getAngle() < -FORWARD_ANGLE &&
                      directionVector.getAngle() >= -SIDE_ANGLE && !isDirZero && !isIdle;

    if (isWander) {
        state = State::WANDER;
    } else if (isFollow) {
        state = State::FOLLOW;
    } else if (isDirLeft) {
        state = State::DIR_LEFT;
    } else if (isDirRight) {
        state = State::DIR_RIGHT;
    } else if (isIdle) {
        state = State::IDLE;
    }

    if (mInfectedForEpochs > parSickFor) {
        agentType = AgentTypeHelper::AgentType::RECOVERED;
    }
    if (agentType == AgentTypeHelper::AgentType::INFECTIOUS && mInfectedForEpochs > mDiesAfterEpochs && isGoingToDie) {
        agentType = AgentTypeHelper::AgentType::DECEASED;
    }

    Action action = mQExploiter->exploit<State, Action>(state);

    switch (agentType) {
        case AgentTypeHelper::AgentType::INFECTIOUS:
            mInfectedForEpochs++;
            mCamera->Disable();
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
            mCamera->Disable();
            break;
    }

    std::array<double, 2> wheelSpeeds = action.getWheelSpeed();
    if (action == Action::TURN_RIGHT) {
        wheelSpeeds[0] = 1.0f;
        wheelSpeeds[1] = 0.0f;
    }
    if (action == Action::TURN_LEFT) {
        wheelSpeeds[0] = 0.0f;
        wheelSpeeds[1] = 1.0f;
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

    LOG << "---------------------------------------------" << std::endl;
    LOG << "Id: " << mId << std::endl;
    LOG << "Type: " << AgentTypeHelper::GetAgentTypeAsString(this->agentType) << std::endl;
    LOG << "Action taken: " << action.getName() << std::endl;
    LOG << "State: " << state.getName() << std::endl;
}

void InfectRandomWalk::Destroy() {
    delete mQExploiter;
}
/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(InfectRandomWalk, "infect_random_walk")
