/*
 * A simple controller for the foot-bot. Obstacle avoidance is implemented
 * The robot moves towards a destination while avoiding the obstacles in its path.
 *
 * This controller is meant to be used with the XML files:
 *  scenes/single_footbot.argos
 */

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <limits>
#include <fstream>

#include "qlearner.hpp"

using namespace argos;

class FootbotQLearnDiy : public CCI_Controller {

public:
    /** Used for initializations. */
    FootbotQLearnDiy();

    /**
     * Caution: Using the destructor is not recommended.
     * Allocate and free all memory in Init() and Destroy()
     */
    virtual ~FootbotQLearnDiy() {}

    /**
     * Function for initialization.
     */
    virtual void Init(TConfigurationNode &t_node);

    /**
     * This contains the logic with which the robot operates.
     */
    virtual void ControlStep();

    /**
     * This function is called right after Init().
     */
    virtual void Reset() {}

    /**
      * The opposite pair of Init()
      */
    virtual void Destroy();

    enum Stage {
        EXPLORE,
        EXPLOIT,
        EXPLORE_EXPLOIT,
        INVALID
    };

    // Small value, close to 0. Used for floating point comparisons to "0".
    constexpr double static EXP_EPSILON = 0.01;

    constexpr int static GOAL_STATE = 3;

    constexpr int static NUM_STATES = 4;

    constexpr int static NUM_ACTIONS = 4;
private:

    static Stage parseStageFromString(const std::string& stageString);

    std::string getActionName(double x, double y);

    ql::QLearner * mQLearner;

    int mPrevState = 0;

    int epoch = 0;

    double globalMaxLightReading;

    /** ACTUATORS AND SENSORS */
    /* Pointer to the differential steering actuator. */
    CCI_DifferentialSteeringActuator *mDiffSteering;

    /* Pointer to the foot-bot proximity sensor. */
    CCI_FootBotProximitySensor *mProximitySensor;

    /* Pointer to the foot-bot light sensor. */
    CCI_FootBotLightSensor *mLightSensor;

    /** PARAMETERS FROM THE ARGOS FILE */

    /* Wheel speed. */
    Real parWheelVelocity;

    // If true then learning phase, if false, then exploit phase.
    Stage parStage;

    // Threshold value for IDLE state
    double parThreshold;

    // Learning rate. Close to 0: nothing new will be learnt, close to 1: the old value will be completely discarded.
    double parLearnRate;

    // Discount factor. If close to zero, then the agent prefers the immediate reward, otherwise the long term reward.
    double parDiscountFactor;
};