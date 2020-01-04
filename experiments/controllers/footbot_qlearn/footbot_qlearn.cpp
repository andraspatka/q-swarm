#include "footbot_qlearn.h"

FootbotQLearn::FootbotQLearn() :
        mDiffSteering(NULL),
        mProximitySensor(NULL),
        mWheelVelocity(30.0f) {}

/**
 * Get the pointer to the actuator and the sensor.
 * Also get the velocity parameter.
 */
void FootbotQLearn::Init(TConfigurationNode &t_node) {

    mDiffSteering = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    mProximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    mLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");

    GetNodeAttributeOrDefault(t_node, "velocity", mWheelVelocity, mWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "explore_exploit", EPSILON, EPSILON);
    initWireFitQLearn();
}

void FootbotQLearn::initWireFitQLearn() {
    // The action that the agent can take is moving, using its differential steering actuator.
    rl::Action minAction = {0.0f, 0.0f};
    rl::Action maxAction = {mWheelVelocity, mWheelVelocity};

    // How many values the state vector can take.
    const int STATE_DIMENSIONS = 12;
    // number of possible discrete values between minAction and maxAction
    const int BASE_OF_DIMENSIONS = 2;
    const int ACTION_LENGTH = 2;

    mLearner = new rl::FidoControlSystem(STATE_DIMENSIONS, minAction, maxAction, BASE_OF_DIMENSIONS);

    // Initialize the backpropagation NN trainer
    double learningRate = 0.05f;
    double momentumTerm = 0.9f; // between 0 and 1, rec: 0.9 tunable: 0.8 - 0.999
    double targetErrorLevel = 0.0025f;
    int maxEpochs = 1000;
    // Todo: Use Adadelta instead? => Adaptive learning rate
    mTrainer = new net::Backpropagation(learningRate, momentumTerm, targetErrorLevel, maxEpochs);
    double rho = 0.25f;
    //mTrainer = new net::Adadelta(rho, targetErrorLevel, maxEpochs);

    // Initialize the Least Square Interpolator
    double smoothingFactor = 0.4;
    double e = 0.1f;
    mLSInterpolator = new rl::LSInterpolator(smoothingFactor, e);

    unsigned int numHiddenLayers = 2;
    unsigned int numNeuronsPerHiddenLayer = 3;
    unsigned int numberOfWires = 12;
    double wireFitQLearningRate = 0.5f;
    double discountFactor = 0.5f; // 0 - immediate reward, 1 - long term reward

    mWireFitQLearner = new rl::WireFitQLearn(STATE_DIMENSIONS, ACTION_LENGTH, numHiddenLayers, numNeuronsPerHiddenLayer,
                                             numberOfWires, minAction, maxAction, BASE_OF_DIMENSIONS, mLSInterpolator,
                                             mTrainer, wireFitQLearningRate, discountFactor);
}

void FootbotQLearn::ControlStep() {
    rl::State states;
    double maxLightReading = 0.0f;
    int i = 1;
    CVector2 vectorSumGoal(0, 0);
    for (auto reading : mLightSensor->GetReadings()) {
        vectorSumGoal += CVector2(reading.Value, reading.Angle);
        if ( i % 2 == 0  && i != 0) {
            states.push_back(vectorSumGoal.Length());
            vectorSumGoal = CVector2(0, 0);
        }
        ++i;
        if (reading.Value > maxLightReading) {
            maxLightReading = reading.Value;
        }
    }

    rl::Action action = mWireFitQLearner->chooseBoltzmanAction(states, EPSILON);

    double rewardValue = maxLightReading;
    if (k == 1) {
        mWireFitQLearner->applyReinforcementToLastAction(rewardValue, states);
        k = 0;
    }
    ++k;

    /*if (action[0] == 0.0f && action[1] == 0.0f) { //{0, 0} action does nothing.
        action[0] = mWheelVelocity;
        action[1] = mWheelVelocity;
    }*/
    //LOG<< "Action taken: "<< action[0] << " " << action[1] << std::endl;
    LOG<< "Reward: " << rewardValue << std::endl;
    mDiffSteering->SetLinearVelocity(action[0], action[1]);
}

void FootbotQLearn::Destroy() {
    delete mLearner;
    delete mWireFitQLearner;
    delete mTrainer;
    delete mLSInterpolator;
}

/**
 * Register the controller.
 * This is needed in order for argos to be able to bind the scene to this controller.
 */
REGISTER_CONTROLLER(FootbotQLearn, "footbot_qlearn_controller")
