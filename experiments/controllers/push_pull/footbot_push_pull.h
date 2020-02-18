/*
 * A simple controller for the foot-bot. Obstacle avoidance is implemented
 * The robot moves towards a destination while avoiding the obstacles in its path.
 *
 * This controller is meant to be used with the XML files:
 *  scenes/push_pull.argos
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

using namespace argos;

class FootbotPushPull : public CCI_Controller {

public:
    /** Used for initializations. */
    FootbotPushPull();

    /**
      * Caution: Using the destructor is not recommended.
     * Allocate and free all memory in Init() and Destroy()
     */
    virtual ~FootbotPushPull() {}

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
    virtual void Destroy() {}

private:

    /* Pointer to the differential steering actuator. */
    CCI_DifferentialSteeringActuator *mDiffSteering;

    /* Pointer to the foot-bot proximity sensor. */
    CCI_FootBotProximitySensor *mProximity;

    /* Pointer to the foot-bot light sensor. */
    CCI_FootBotLightSensor *mLightSensor;

    /* Wheel speed. */
    Real mWheelVelocity;

    /* Turn left value. */
    Real mLeftWheelVelocity;

    /* Turn right value. */
    Real mRightWheelVelocity;

    /* Acceptable distance to the nearest obstacle. */
    Real mMinDistance;
};