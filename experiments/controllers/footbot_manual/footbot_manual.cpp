/* Include the controller definition */
#include "footbot_manual.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <monitoring/logger.hpp>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

CFootBotManualControl::CFootBotManualControl() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL) {}

void CFootBotManualControl::Init(TConfigurationNode& t_node) {
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   mPosition = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
   m_pcLEDs->SetSingleColor(12, CColor::WHITE);
   GetNodeAttribute(t_node, "max_speed", this->parMaxSpeed);
   ql::Logger::clearMyLogs(this->m_strId);
}

void CFootBotManualControl::ControlStep() {
   /* Follow the control vector only if selected */
   if(m_bSelected) {
       this->m_pcWheels->SetLinearVelocity(this->diffSteeringVals[0], this->diffSteeringVals[1]);
   }
   auto position = mPosition->GetReading().Position;
   ql::Logger::log(this->m_strId, {std::to_string(position.GetX()), std::to_string(position.GetY())});
}

void CFootBotManualControl::Select() {
   m_bSelected = true;
   m_pcLEDs->SetSingleColor(12, CColor::RED);
}

void CFootBotManualControl::Deselect() {
   m_bSelected = false;
   m_pcLEDs->SetSingleColor(12, CColor::WHITE);
}

void CFootBotManualControl::SetDiffSteering(double diffSteer[2]) {
    if (diffSteer[0] == 0.0f && diffSteer[1] == 0.0f) {
        m_pcLEDs->SetSingleColor(12, CColor::PURPLE);
    } else {
        m_pcLEDs->SetSingleColor(12, CColor::RED);
    }
    this->diffSteeringVals[0] = diffSteer[0] * parMaxSpeed;
    this->diffSteeringVals[1] = diffSteer[1] * parMaxSpeed;
}
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotManualControl, "footbot_manualcontrol")
