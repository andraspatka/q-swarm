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

#include "thread_safe_random.hpp"

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

    // Small value, close to 0. Used for floating point comparisons to "0".
    constexpr double static EXP_EPSILON = 0.01;

    constexpr int static GOAL_STATE = 4;

    constexpr int static NUM_STATES = 4;

    constexpr int static NUM_ACTIONS = 4;

    constexpr double static DISCOUNT_FACTOR = 0.8f;
private:

    std::string getActionName(double x, double y);

    int train(int state, int nextState);

    int exploit(int state);

    void printQ(const std::string& fileName);

    void readQ(const std::string& fileName);

    int mPrevState = 0;

    std::vector<std::vector<double>> Q;
    std::vector<std::vector<double>> R;


    /* Pointer to the differential steering actuator. */
    CCI_DifferentialSteeringActuator *mDiffSteering;

    /* Pointer to the foot-bot proximity sensor. */
    CCI_FootBotProximitySensor *mProximitySensor;

    /* Pointer to the foot-bot light sensor. */
    CCI_FootBotLightSensor *mLightSensor;

    /* Wheel speed. */
    Real mWheelVelocity;

    int epoch = 0;
    // Exploration constant. Defines how much the agent should exploit vs explore.
    // This value might be overridden from the argos file
    double exploreExploit;

    bool isTraining;

    // Threshold value for IDLE state
    double mThreshold;

    double maxReward;

    void initQLearn();
};