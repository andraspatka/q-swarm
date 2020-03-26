#include "footbot_follow.h"

FootbotFollow::FootbotFollow() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        globalMinCameraBlobDist(0),
        epoch(0) {}

/**
 *
 *            STOP    TURN_LEFT   TURN_RIGHT  FORWARD
 * 0 WANDER     -1      0           0           0.1
 * 1 FOLLOW     -1      0           0           1
 * 2 UTURN      -1      0           0           0
 * 3 DIR_LEFT   -1      0.1         0           0
 * 4 DIR_RIGHT  -1      0           0.1         0
 * 5 IDLE        1      0           0           0
 */
void FootbotFollow::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mLed = GetActuator<CCI_LEDsActuator>("leds");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    mCamera->Enable();
    std::string parStageString;

    GetNodeAttribute(t_node, "velocity", parWheelVelocity);
    GetNodeAttribute(t_node, "learning_rate", parLearnRate);
    GetNodeAttribute(t_node, "discount_factor", parDiscountFactor);
    GetNodeAttribute(t_node, "threshold", parThreshold);
    GetNodeAttribute(t_node, "stage", parStageString);
    GetNodeAttribute(t_node, "qmat_filename", parQMatFileName);

    parStage = parseStageFromString(parStageString);
    mQLearner = new QLearner(NUM_STATES, NUM_ACTIONS, parDiscountFactor, parLearnRate);
    std::vector<std::tuple<int, int>> impossibleStates = {
            std::make_tuple(0, 0), // WANDER state, STOP action
            std::make_tuple(1, 0), // FOLLOW state, STOP action
            std::make_tuple(2, 0), // UTURN, STOP action
            std::make_tuple(3, 0), // DIR_LEFT, STOP action
            std::make_tuple(4, 0)  // DIR_RIGHT, STOP action
    };
    std::vector<std::tuple<int, int, double>> rewards = {
            std::make_tuple(0, 3, 0.1), // WANDER state, FORWARD action
            std::make_tuple(1, 3, 1), // FOLLOW state, FORWARD action
            std::make_tuple(3, 1, 0.1), // DIR_LEFT state, TURN_LEFT action
            std::make_tuple(4, 2, 0.1), // DIR_RIGHT state, TURN_LEFT action
            std::make_tuple(5, 0, 1), // IDLE state, STOP action
    };
    mQLearner->initR(impossibleStates, rewards);
    if (parStage == Stage::EXPLOIT) {
        mQLearner->readQ("qmats/" + parQMatFileName);
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
 * Function for aggregating multiple sensor values: max
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

            fpullVector += QLMathUtils::readingToVector(r->Distance, r->Angle, A, B_PULL, C_PULL,
                                                            QLMathUtils::cameraToDistance);
        }
    }

    // Left front values
    for (int i = 0; i <= 4; ++i) {
        fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                        C_PUSH, QLMathUtils::proxToDistance);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 19; i <= 23; ++i) {
        fpushVector += QLMathUtils::readingToVector(proxReadings.at(i).Value, proxReadings.at(i).Angle, A, B_PUSH,
                                                        C_PUSH, QLMathUtils::proxToDistance);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }

    CVector2 directionVector = fpullVector - fpushVector;
    bool isDirZero = QLMathUtils::closeToZero(directionVector.Length());

    // TODO: revise wander and idle state
    bool isWander = QLMathUtils::closeToZero(maxProx) && minDistanceBlob.Distance == 1000.0f;
    bool isFollow = QLMathUtils::absAngleInDegrees(directionVector.Angle()) < 30.0f && !isDirZero;
    bool isUturn = QLMathUtils::absAngleInDegrees(directionVector.Angle()) > 125.0f && !isDirZero;
    bool isDirLeft = QLMathUtils::angleInDegrees(directionVector.Angle()) > 30.0f &&
                     QLMathUtils::angleInDegrees(directionVector.Angle()) < 125.0f && !isDirZero;
    bool isDirRight = QLMathUtils::angleInDegrees(directionVector.Angle()) < -30.0f &&
                      QLMathUtils::angleInDegrees(directionVector.Angle()) > -125.0f && !isDirZero;
    bool isIdle = isDirZero && minDistanceBlob.Distance != 1000.0f;

    std::string actualState;
    // States
    if (isWander) {
        actualState = "WANDER";
        state = 0;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isFollow) {
        actualState = "FOLLOW";
        state = 1;
        mLed->SetAllColors(CColor::YELLOW);
    } else if (isUturn) {
        actualState = "UTURN";
        state = 2;
        mLed->SetAllColors(CColor::PURPLE);
    } else if (isDirLeft) {
        actualState = "DIR_LEFT";
        state = 3;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isDirRight) {
        actualState = "DIR_RIGHT";
        state = 4;
        mLed->SetAllColors(CColor::WHITE);
    } else if (isIdle) {
        actualState = "IDLE";
        state = 5;
        mLed->SetAllColors(CColor::GREEN);
    }

    if (state == -1) {
        int bp = 0;
    }

    if (isFollow + isWander + isUturn + isDirLeft + isDirRight + isIdle > 1) {
        int bp = 0;
    }

    epoch++;
    if (mQLearner->getLearningRate() > 0.05f && epoch % 175 == 0 && parStage == Stage::TRAIN) {
        mQLearner->setLearningRate(mQLearner->getLearningRate() - 0.05f);
    }
    int actionIndex = mQLearner->train(mPrevState, state);

    mPrevState = state;

    std::array<double, 2> action = QLUtils::getActionFromIndex(actionIndex, parWheelVelocity);
    mDiffSteering->SetLinearVelocity(action[0], action[1]);
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "Stage: " << parStage << std::endl;
    LOG << "fpush: " << fpushVector << std::endl;
    LOG << "fpull: " << fpullVector << std::endl;
    LOG << "Direction: " << directionVector << std::endl;

    LOG << "Action taken: " << QLUtils::getActionName(action[0], action[1]) << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "Learning rate: " << mQLearner->getLearningRate() << std::endl;
    LOG << "Global min camera: " << globalMinCameraBlobDist << std::endl;
}

void FootbotFollow::Destroy() {
    mQLearner->printQ("qmats/" + parQMatFileName);
    delete mQLearner;
}

FootbotFollow::Stage FootbotFollow::parseStageFromString(const std::string &stageString) {
    if (stageString == "train") return FootbotFollow::Stage::TRAIN;
    if (stageString == "exploit") return FootbotFollow::Stage::EXPLOIT;
    std::cerr << "Invalid Stage value: " << stageString;
    exit(1);
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotFollow, "footbot_follow")
