#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <limits>
#include <fstream>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

#include <qlearner/ql_utils.hpp>
#include <qlearner/stage.hpp>
#include <qlearner/ql_math_utils.hpp>
#include <qlearner/qlearner.hpp>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <qlearner/qexploiter.hpp>

using namespace argos;
using namespace ql;

class FootbotLeader : public CCI_Controller {

public:
    /** Used for initializations. */
    FootbotLeader();

    /**
     * Caution: Using the destructor is not recommended.
     * Allocate and free all memory in Init() and Destroy()
     */
    virtual ~FootbotLeader() {}

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

private:
    static const int NUM_STATES = 5;
    static const int NUM_ACTIONS = 4;
    static const int STATE_THRESHOLD = 40;

    // The push gauss curve's centre point is the robot
    static constexpr double B_PUSH = 0.0f;
    // The pull gauss curve's centre point is the prox sensor's coverage limit
    static constexpr double B_PULL = 2.5f;
    // Width of the gauss curve for pushing forces
    static constexpr double C_PUSH = 0.5f;
    // Width of the gauss curve for pulling forces
    static constexpr double C_PULL = 0.6f;
    // Height of the gauss curve
    static constexpr double A = 1.0f;

    static constexpr double FORWARD_ANGLE = 20.0f;
    static constexpr double SIDE_ANGLE = 180.0f;

    ql::QLearner * mQLearner;
    ql::QExploiter * mQExploiter;

    int mPrevState = 0;

    int epoch = 0;

    double mGlobalMaxLightReading = 0.0f;

    int mLearnedEpoch = 4000;

    std::array<int, NUM_STATES> mStateStats;

    /** ACTUATORS AND SENSORS */
    /* Pointer to the differential steering actuator. */
    CCI_DifferentialSteeringActuator *mDiffSteering;

    /* Pointer to the LED actuator */
    CCI_LEDsActuator *mLed;

    /* Pointer to the foot-bot proximity sensor. */
    CCI_FootBotProximitySensor *mProximitySensor;

    /* Pointer to the foot-bot light sensor. */
    CCI_FootBotLightSensor *mLightSensor;

    /* Pointer to the footbot positioning sensor. */
    CCI_PositioningSensor* mPosition;

    /** PARAMETERS FROM THE ARGOS FILE */

    /* Wheel speed. */
    Real parWheelVelocity;

    // If true then learning phase, if false, then exploit phase.
    StageHelper::Stage parStage;

    // Threshold value for IDLE state
    double parThreshold;

    // Learning rate. Close to 0: nothing new will be learnt, close to 1: the old value will be completely discarded.
    double parLearnRate;

    // Discount factor. If close to zero, then the agent prefers the immediate reward, otherwise the long term reward.
    double parDiscountFactor;
};