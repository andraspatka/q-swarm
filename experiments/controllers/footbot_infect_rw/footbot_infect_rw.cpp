#include "footbot_infect_rw.h"

InfectRandomWalk::InfectRandomWalk() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
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
void InfectRandomWalk::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    mPosition = GetSensor<CCI_PositioningSensor>("positioning");

    mCamera->Enable();
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "infectious", parNoOfInfectious);
    GetNodeAttribute(t_node, "infect_prob", parInfectionProb);

    mQExploiter = new QExploiter(NUM_STATES, NUM_ACTIONS);
    mQExploiter->readQ("qmats/follow-train.qlmat");
    InitInfectious();
}

void InfectRandomWalk::Reset() {
    InitInfectious();
}

void InfectRandomWalk::InitInfectious() {
    ql::Logger::clearMyLogs(this->m_strId);
    int idNumber = std::stoi(this->m_strId);
    mInfectedForEpochs = 0;
    if (idNumber < parNoOfInfectious) {
        agentType = AGENT_TYPE::INFECTIOUS;
    } else {
        agentType = AGENT_TYPE::SUSCEPTIBLE;
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
    ql::Vector fpushVector;
    ql::Vector fpullVector;

    double maxProx = 0.0f;

    State state;

    auto proxReadings = mProximitySensor->GetReadings();
    auto cameraReadings = mCamera->GetReadings().BlobList;

    CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob minDistanceBlob(CColor::WHITE, CRadians::TWO_PI, 1000.0f);
    for (auto r : cameraReadings) {
        if (ql::QLMathUtils::cameraToDistance(r->Distance) <= 1 && r->Color == CColor::RED && agentType == SUSCEPTIBLE) {
            if (drand48() < parInfectionProb) {
                agentType = INFECTIOUS;
            }
        }
    }

    for (int i = 0; i <= 23; ++i) {
        if (!QLMathUtils::closeToZero(proxReadings.at(i).Value)) {
            fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                        C_PUSH, QLMathUtils::proxToDistance);
            maxProx = std::max(maxProx, proxReadings.at(i).Value);
        }
    }

    ql::Vector directionVector = -fpushVector;
    bool isDirZero = directionVector.isZero();

    bool isWander = isDirZero;
    bool isFollow = directionVector.getAbsAngle() <= FORWARD_ANGLE && !isDirZero;
    bool isDirLeft = directionVector.getAngle() > FORWARD_ANGLE &&
                     directionVector.getAngle() <= SIDE_ANGLE && !isDirZero;
    bool isDirRight = directionVector.getAngle() < -FORWARD_ANGLE &&
                      directionVector.getAngle() >= -SIDE_ANGLE && !isDirZero;

    if (isWander) {
        state = State::WANDER;
    } else if (isFollow) {
        state = State::FOLLOW;
    } else if (isDirLeft) {
        state = State::DIR_LEFT;
    } else if (isDirRight) {
        state = State::DIR_RIGHT;
    }

    if (mInfectedForEpochs > 100) {
        agentType = REMOVED;
    }

    Action action = mQExploiter->exploit(state);

    CColor agentColor = CColor::WHITE;
    switch (agentType) {
        case INFECTIOUS:
            agentColor = CColor::RED;
            mInfectedForEpochs++;
            break;
        case SUSCEPTIBLE:
            agentColor = CColor::CYAN;
            break;
        case REMOVED:
            agentColor = CColor::GRAY50;
            action = Action::STOP;
            mCamera->Disable();
            break;
    }
    mLed->SetAllColors(agentColor);

    epoch++;

    std::array<double, 2> wheelSpeeds = action.getWheelSpeed();
    wheelSpeeds[0] *= parWheelVelocity;
    wheelSpeeds[1] *= parWheelVelocity;

    mDiffSteering->SetLinearVelocity(wheelSpeeds[0], wheelSpeeds[1]);

    const CVector3 actualPosition = this->mPosition->GetReading().Position;
    std::vector<std::string> toLog = {
            std::to_string(actualPosition.GetX()),
            std::to_string(actualPosition.GetY()),
            state.getName(),
            action.getName(),
            getAgentTypeAsString()
    };
    ql::Logger::log(this->m_strId, toLog, true);
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "Id: " << this->m_strId << std::endl;
    LOG << "Type: " << getAgentTypeAsString() << std::endl;
    LOG << "Action taken: " << action.getName() << std::endl;
    LOG << "State: " << state.getName() << std::endl;
}

void InfectRandomWalk::Destroy() {
    delete mQExploiter;
}

std::string InfectRandomWalk::getAgentTypeAsString() {
    switch (this->agentType) {
        case INFECTIOUS:
            return "INFECTIOUS";
        case SUSCEPTIBLE:
            return "SUSCEPTIBLE";
        case REMOVED:
            return "REMOVED";
    }
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(InfectRandomWalk, "infect_random_walk")
