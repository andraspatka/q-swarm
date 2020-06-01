#include "manual_qtuser_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <QKeyEvent>

CManualControlQTUserFunctions::CManualControlQTUserFunctions() :
        m_pcController(NULL) {
    /* No key is pressed initially */
    m_punPressedKeys[DIRECTION_FORWARD] = 0;
    m_punPressedKeys[DIRECTION_LEFT] = 0;
    m_punPressedKeys[DIRECTION_RIGHT] = 0;
    RegisterUserFunction<CManualControlQTUserFunctions,CFootBotEntity>(&CManualControlQTUserFunctions::Draw);
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
        case Qt::Key_M:
            m_punPressedKeys[DIRECTION_STOP] = 1;
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
        case Qt::Key_K:
            /* Stop */
            m_punPressedKeys[DIRECTION_STOP] = 0;
            SetDirectionFromKeyEvent();
            break;
        default:
            /* Unknown key */
            GetQTOpenGLWidget().KeyReleased(pc_event);
            break;
    }
}

void CManualControlQTUserFunctions::Draw(CFootBotEntity& c_entity) {
    /* The position of the text is expressed wrt the reference point of the footbot
     * For a foot-bot, the reference point is the center of its base.
     * See also the description in
     * $ argos3 -q foot-bot
     */
    DrawText(CVector3(0.0, 0.0, 0.3),   // position
             c_entity.GetId().c_str(), CColor::RED); // text
}

void CManualControlQTUserFunctions::EntitySelected(CEntity &c_entity) {
    /* Make sure the entity is a foot-bot */
    CFootBotEntity *pcFB = dynamic_cast<CFootBotEntity *>(&c_entity);
    if (!pcFB) return;
    /* It's a foot-bot; extract its controller */
    m_pcController = dynamic_cast<CFootBotManualControl *>(&pcFB->GetControllableEntity().GetController());
    if (!m_pcController) {
        return;
    }

    /* Tell that foot-bot that it is selected */
    m_pcController->Select();
    /* Reset key press information */
    m_punPressedKeys[DIRECTION_FORWARD] = 0;
    m_punPressedKeys[DIRECTION_LEFT] = 0;
    m_punPressedKeys[DIRECTION_RIGHT] = 0;
    m_punPressedKeys[DIRECTION_STOP] = 0;
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
    double diffSteering[2] = {0, 0};
    if (m_punPressedKeys[DIRECTION_FORWARD]) {
        diffSteering[0] += 1;
        diffSteering[1] += 1;
    }
    if (m_punPressedKeys[DIRECTION_LEFT]) {
        diffSteering[0] += 0;
        diffSteering[1] += 1;
    }
    if (m_punPressedKeys[DIRECTION_RIGHT]) {
        diffSteering[0] += 1;
        diffSteering[1] += 0;
    }
    if (m_punPressedKeys[DIRECTION_STOP]) {
        diffSteering[0] += 0;
        diffSteering[1] += 0;
    }

    m_pcController->SetDiffSteering(diffSteering);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CManualControlQTUserFunctions, "manual_qtuser_functions")
