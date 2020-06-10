#include "quarantine_qtuser_functions.h"

/****************************************/
/****************************************/

QuarantineQTUserFunctions::QuarantineQTUserFunctions() {
    RegisterUserFunction<QuarantineQTUserFunctions,CLightEntity>(&QuarantineQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void QuarantineQTUserFunctions::Draw(CLightEntity& c_light) {
    /* The position of the text is expressed wrt the reference point of the footbot
     * For a foot-bot, the reference point is the center of its base.
     * See also the description in
     * $ argos3 -q foot-bot
     */
    QFont font = QFont();
    font.setFamily("Helvetica [Cronyx]");
    font.setPixelSize(20);
    font.setCapitalization(QFont::Capitalize);

    DrawText(CVector3(0.4, 0.5, 0.3),   // position
             c_light.GetId().c_str(), CColor::BLACK, font); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(QuarantineQTUserFunctions, "quarantine_qtuser_functions")
