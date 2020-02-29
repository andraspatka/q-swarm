#include "manual_qtuser_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <QKeyEvent>

CManualControlQTUserFunctions::CManualControlQTUserFunctions() :
        m_pcController(NULL) {
    /* No key is pressed initially */
    m_punPressedKeys[DIRECTION_FORWARD] = 0;
    m_punPressedKeys[DIRECTION_BACKWARD] = 0;
    m_punPressedKeys[DIRECTION_LEFT] = 0;
    m_punPressedKeys[DIRECTION_RIGHT] = 0;
}

void CManualControlQTUserFunctions::KeyPressed(QKeyEvent *pc_event) {
    /* Make sure that a controller was set */
    if (!m_pcController) {
        GetQTOpenGLWidget().KeyPressed(pc_event);
        return;
    }
    switch (pc_event->key()) {
        case Qt::Key_I:
            /* Forwards */
            m_punPressedKeys[DIRECTION_FORWARD] = 1;
            SetDirectionFromKeyEvent();
            break;
        case Qt::Key_K:
            /* Backwards */
            m_punPressedKeys[DIRECTION_BACKWARD] = 1;
            SetDirectionFromKeyEvent();
            break;
        case Qt::Key_J:
            /* Left */
            m_punPressedKeys[DIRECTION_LEFT] = 1;
            SetDirectionFromKeyEvent();
            break;
        case Qt::Key_L:
            /* Right */
            m_punPressedKeys[DIRECTION_RIGHT] = 1;
            SetDirectionFromKeyEvent();
            break;
        default:
            /* Unknown key */
            GetQTOpenGLWidget().KeyPressed(pc_event);
            break;
    }
}

void CManualControlQTUserFunctions::KeyReleased(QKeyEvent *pc_event) {
    /* Make sure that a controller was set */
    if (!m_pcController) {
        GetQTOpenGLWidget().KeyReleased(pc_event);
        return;
    }
    switch (pc_event->key()) {
        case Qt::Key_I:
            /* Forwards */
            m_punPressedKeys[DIRECTION_FORWARD] = 0;
            SetDirectionFromKeyEvent();
            break;
        case Qt::Key_K:
            /* Backwards */
            m_punPressedKeys[DIRECTION_BACKWARD] = 0;
            SetDirectionFromKeyEvent();
            break;
        case Qt::Key_J:
            /* Left */
            m_punPressedKeys[DIRECTION_LEFT] = 0;
            SetDirectionFromKeyEvent();
            break;
        case Qt::Key_L:
            /* Right */
            m_punPressedKeys[DIRECTION_RIGHT] = 0;
            SetDirectionFromKeyEvent();
            break;
        default:
            /* Unknown key */
            GetQTOpenGLWidget().KeyReleased(pc_event);
            break;
    }
}

void CManualControlQTUserFunctions::EntitySelected(CEntity &c_entity) {
    /* Make sure the entity is a foot-bot */
    CFootBotEntity *pcFB = dynamic_cast<CFootBotEntity *>(&c_entity);
    if (!pcFB) return;
    /* It's a foot-bot; extract its controller */
    m_pcController = dynamic_cast<CFootBotManualControl *>(&pcFB->GetControllableEntity().GetController());
    /* Tell that foot-bot that it is selected */
    m_pcController->Select();
    /* Reset key press information */
    m_punPressedKeys[DIRECTION_FORWARD] = 0;
    m_punPressedKeys[DIRECTION_BACKWARD] = 0;
    m_punPressedKeys[DIRECTION_LEFT] = 0;
    m_punPressedKeys[DIRECTION_RIGHT] = 0;
}

void CManualControlQTUserFunctions::EntityDeselected(CEntity &c_entity) {
    /* Make sure that a controller was set (should always be true...) */
    if (!m_pcController) return;
    /* Tell the foot-bot that it is deselected */
    m_pcController->Deselect();
    /* Forget the controller */
    m_pcController = NULL;
}

void CManualControlQTUserFunctions::SetDirectionFromKeyEvent() {
    double diffSteering[2];
    if (m_punPressedKeys[DIRECTION_FORWARD]) {
        diffSteering[0] = 1;
        diffSteering[1] = 1;
    }
    if (m_punPressedKeys[DIRECTION_BACKWARD]) {
        diffSteering[0] = -1;
        diffSteering[1] = -1;
    }
    if (m_punPressedKeys[DIRECTION_LEFT]) {
        diffSteering[0] = 0;
        diffSteering[1] = 1;
    }
    if (m_punPressedKeys[DIRECTION_RIGHT]) {
        diffSteering[0] = 1;
        diffSteering[1] = 0;
    }

    m_pcController->SetDiffSteering(diffSteering);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CManualControlQTUserFunctions, "manual_qtuser_functions")
