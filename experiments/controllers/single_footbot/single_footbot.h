/*
 * A simple controller for the foot-bot. Currently it is not quite smart, as it always goes forward.
 *
 * This controller is meant to be used with the XML files:
 *  scenes/footbot_1.argos
 */

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

using namespace argos;

class SingleFootBot : public CCI_Controller {

public:
   
   /** Used for initializations. */
   SingleFootBot();

   /** 
    * Caution: Using the destructor is not recommended.
    * Allocate and free all memory in Init() and Destroy()
    */
   virtual ~SingleFootBot() {}

   /**
    * Function for initialization.
    */
   virtual void Init(TConfigurationNode& t_node);
   
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

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* mDiffSteering;
   
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* mProximity; 

   /* Wheel speed. */
   Real mWheelVelocity;
};