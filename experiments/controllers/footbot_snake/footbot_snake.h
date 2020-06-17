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
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <qlearner/qlearner.hpp>
#include <potnavi/math_utils.hpp>
#include <potnavi/polar_vector.hpp>
#include <qlearner/stage.hpp>
#include <qlearner/follower_action.hpp>
#include <qlearner/follower_state.hpp>

#include <monitoring/logger.hpp>
#include <qlearner/qexploiter.hpp>

using namespace argos;
using namespace ql;

class FootbotSnake : public CCI_Controller {

public:
    /** Used for initializations. */
    FootbotSnake();

    /**
     * Caution: Using the destructor is not recommended.
     * Allocate and free all memory in Init() and Destroy()
     */
    virtual ~FootbotSnake() {}

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
    constexpr int static NUM_STATES = 4;

    constexpr int static NUM_ACTIONS = 4;

    constexpr int static STATE_THRESHOLD = 40;

    constexpr int static PROX_READING_PER_SIDE = 5;

    // The push gauss curve's centre point is the robot
    constexpr double static B_PUSH = 0.0f;
    // The pull gauss curve's centre point is the prox sensor's coverage limit
    constexpr double static B_PULL = 3.0f;
    // Width of the gauss curve for pushing forces
    constexpr double static C_PUSH = 0.5;
    // Width of the gauss curve for pulling forces
    constexpr double static C_PULL = 0.6f;
    // Height of the gauss curve
    constexpr double static A = 1.0f;

    QLearner * mQLearner;

    QExploiter * mQExploiter;

    FollowerState mPrevState = FollowerState::NO_TARGET_TO_FOLLOW;

    int epoch = 0;

    int mLearnedEpoch = 10000;

    double maxCamera = 0;

    std::array<int, NUM_STATES> mStateStats;

    /** ACTUATORS AND SENSORS */
    /* Pointer to the differential steering actuator. */
    CCI_DifferentialSteeringActuator *mDiffSteering;

    /* Pointer to the LED actuator */
    CCI_LEDsActuator *mLed;

    /* Pointer to the foot-bot proximity sensor. */
    CCI_FootBotProximitySensor *mProximitySensor;

    /* Pointer to the omnidirectional camera sensor */
    CCI_ColoredBlobOmnidirectionalCameraSensor *mCamera;

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

    bool parShouldLog;
};