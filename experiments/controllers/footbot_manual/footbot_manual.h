/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller to manually control the foot-bot.

 * To control a foot-bot, you need to Shift-Click on it in the OpenGL
 * visualization to select it.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/manualcontrol.argos
 */

#ifndef FOOTBOT_MANUALCONTROL_H
#define FOOTBOT_MANUALCONTROL_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <string>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotManualControl : public CCI_Controller {

public:

   /* Class constructor. */
   CFootBotManualControl();

   /* Class destructor. */
   virtual ~CFootBotManualControl() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_manualcontrol_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}

   /*
    * Sets the selected flag on this robot.
    * When selected, a robot follows the control vector.
    */
   void Select();

   /*
    * Unsets the selected flag on this robot.
    * When unselected, a robot stays still.
    */
   void Deselect();

   /*
    * Sets the control vector.
    */
   void SetDiffSteering(double [2]);

protected:


private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;

    /* Maximum wheel speed */
    Real parMaxSpeed;

   /* Flag to know whether this robot is selected */
   bool m_bSelected;

   /* The control vector */
   double diffSteeringVals[2];
   CCI_PositioningSensor *mPosition;
};

#endif
