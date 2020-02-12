#include "footbot_qlearn_diy.h"

FootbotQLearnDiy::FootbotQLearnDiy() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        mWheelVelocity(30.0f) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void FootbotQLearnDiy::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "threshold", mThreshold, mThreshold);
    GetNodeAttributeOrDefault(t_node, "train", isTraining, isTraining);
//    GetNodeAttributeOrDefault(t_node, "explore_exploit", exploreExploit, exploreExploit);
    exploreExploit = 0.9f;
    mThreshold = 0.25f;
    maxReward = 0.0f;
    epoch = 0;
    initQLearn();
    if (!isTraining) {
        readQ("Qmat.txt");
    }

}

void FootbotQLearnDiy::initQLearn() {
    R = {
            {-1, 0, 0, 1},
            {-1, 0, 0, 0},
            {-1, 0, 0, 0},
            {1, 0, 0, 0}
    };

    Q = {
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0}
    };
}

int FootbotQLearnDiy::train(int state, int nextState) {
    ThreadSafeRandom threadSafeRandom(0, NUM_ACTIONS);
    int possibleAction;
    do {
        possibleAction = threadSafeRandom.getRandomNumber();
    } while (R.at(state).at(possibleAction) == -1);
    double nextStateMaxValue = 0;
    for (int i = 0; i < NUM_ACTIONS; ++i) {
        nextStateMaxValue = std::max(nextStateMaxValue, Q.at(nextState).at(i));
    }
    Q.at(state).at(possibleAction) = R.at(state).at(possibleAction) + DISCOUNT_FACTOR * nextStateMaxValue;
    return possibleAction;
}

int FootbotQLearnDiy::exploit(int state) {
    int actionWithMaxQ = 0;
    double maxQValue = 0;
    for (int i = 0; i < NUM_ACTIONS; ++i) {
        if (Q.at(state).at(i) > maxQValue) {
            maxQValue = Q.at(state).at(i);
            actionWithMaxQ = i;
        }
    }
    return actionWithMaxQ;
}


void FootbotQLearnDiy::printQ(const std::string& fileName) {
    std::ofstream file;
    file.open (fileName);
    for (int i = 0; i < 4; ++i) {
        file << "\n";
        for (int j = 0; j < 4; ++j) {
            file << Q.at(i).at(j) << " ";
        }
    }
    file.close();
}

void FootbotQLearnDiy::readQ(const std::string& fileName) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "There was a problem opening the input file!\n";
        exit(1);//exit or do additional error checking
    }

    for (int i = 0; i < NUM_STATES; ++i) {
        for (int j = 0; j < NUM_ACTIONS; ++j) {
            file >> Q.at(i).at(j);
        }
    }
    file.close();
}

bool closeToZero(double value) {
    return value < FootbotQLearnDiy::EXP_EPSILON;
}

std::string FootbotQLearnDiy::getActionName(double x, double y) {
    if (x == 0.0 && y == 0.0) return "STOP";
    if (x == mWheelVelocity && y == mWheelVelocity) return "FORWARD";
    if (x == 0.0 && y == mWheelVelocity) return "TURN LEFT";
    if (x == mWheelVelocity && y == 0.0f) return "TURN RIGHT";
    return "INVALID";
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
 * Aggregate of sensor values:
 *      f:  0 23
 *      lm: 5 6
 *      rm: 17 18
 *      lf: 1 2 3 4
 *      rf: 19 20 21 22
 *      b:  7 8 9 10 11 12 13 14 15 16
 *
 * States:
 *      WANDER - explorational phase
 *          reward: 0.1
 *          sensors:
 *              prox: 0
 *              light: b=0
 *      IDLE - end destination reached
 *          reward: 1
 *          sensors:
 *              prox: ?
 *              light: max(sensor values) > THRESHOLD
 *      TURN - facing in the wrong direction
 *          reward: -0.2
 *          sensors:
 *              prox: ?
 *              light: b > 0 && front == 0
 *      AVOID - obstacle detected
 *          reward: -1
 *          sensors:
 *              prox: != 0
 *              light: ?
 */
void FootbotQLearnDiy::ControlStep() {
    double maxFrontLight = 0.0f;
    double maxLight = 0.0f;
    double backMaxLight = 0.0f;
    double turningBackMaxLight = 0.0f;

    double maxProx = 0.0f;
    double backMaxProx = 0.0f;

    double rewardValue = 0;

    int state = 0;

    double action[2];

    auto lightReadings = mLightSensor->GetReadings();
    auto proxReadings = mProximitySensor->GetReadings();

    // back
    for (int i = 10; i <= 13; ++i) {
        backMaxProx = std::max(backMaxProx, proxReadings.at(i).Value);
        backMaxLight = std::max(backMaxLight, lightReadings.at(i).Value);
    }

    for (int i = 5; i <= 8; ++i) {
        turningBackMaxLight = std::max(turningBackMaxLight, lightReadings.at(i).Value);
        turningBackMaxLight = std::max(turningBackMaxLight, lightReadings.at(i + 10).Value);
    }

    // Left front values
    for (int i = 0; i <= 4; ++i) {
        maxFrontLight = std::max(maxFrontLight, lightReadings.at(i).Value);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }
    // Right front values
    for (int i = 19; i <= 23; ++i) {
        maxFrontLight = std::max(maxFrontLight, lightReadings.at(i).Value);
        maxProx = std::max(maxProx, proxReadings.at(i).Value);
    }
    for (int i = 0; i < 23; ++i) {
        maxLight = std::max(maxLight, lightReadings.at(i).Value);
    }

    // States
    if (closeToZero(backMaxLight) && closeToZero(maxProx) && closeToZero(backMaxProx) && maxLight < mThreshold) {
         // WANDER state
        state = 0;
    }
    if (backMaxLight > 0.01 && closeToZero(maxProx) && closeToZero(backMaxProx) && maxLight < mThreshold) {
         // TURN state
        state = 1;
    }
    if (maxProx > 0 || backMaxProx > 0) {
         // AVOID state
        state = 2;
    }
    if (maxLight > mThreshold && closeToZero(maxProx)) {
         // IDLE state
         state = 3;
    }

    epoch++;
    if (exploreExploit > 0.3f && epoch % 50 == 0) { // Every 250 epochs it decreases the exploreExploit parameter
        exploreExploit -= 0.05f;
    }

    if (maxReward < rewardValue) {
        maxReward = rewardValue;
    }
    int actionIndex = 0;
    actionIndex = (isTraining) ? train(mPrevState, state) : exploit(state);

    mPrevState = state;
    switch (actionIndex) {
        case 0:
            action[0] = 0;
            action[1] = 0;
            break;
        case 1:
            action[0] = 0;
            action[1] = mWheelVelocity;
            break;
        case 2:
            action[0] = mWheelVelocity;
            action[1] = 0;
            break;
        case 3:
            action[0] = mWheelVelocity;
            action[1] = mWheelVelocity;
            break;
        default:
            action[0] = 0;
            action[1] = 0;
            break;
    }

    mDiffSteering->SetLinearVelocity(action[0], action[1]);

    std::string actualState;
    switch (state) {
        case 0: {
            actualState = "WANDER";
            break;
        }
        case 1: {
            actualState = "TURN";
            break;
        }
        case 2: {
            actualState = "AVOID";
            break;
        }
        case 3: {
            actualState = "IDLE";
            break;
        }
        default: {
            actualState = "ERROR";
        }
    }
    // LOGGING
    LOG << "---------------------------------------------" << std::endl;
    LOG << "BackwProx: " << backMaxProx << std::endl;
    LOG << "MaxProx: " << maxProx << std::endl;
    LOG << "BackwLight: " << backMaxLight << std::endl;
    LOG << "TurningLight: " << turningBackMaxLight << std::endl;
    LOG << "MaxFrontLight: " << maxFrontLight << std::endl;
    LOG << "MaxLight: " << maxLight << std::endl;

    LOG << "Action taken: " << getActionName(action[0], action[1]) << std::endl;
    LOG << "Reward: " << rewardValue << std::endl;
    LOG << "State: " << actualState << std::endl;
    LOG << "ExploreExploit: " << exploreExploit << std::endl;
    LOG << "Max reward: " << maxReward << std::endl;
}

void FootbotQLearnDiy::Destroy() {
    printQ("Qmat.txt");
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearnDiy, "footbot_qlearn_diy_controller")
