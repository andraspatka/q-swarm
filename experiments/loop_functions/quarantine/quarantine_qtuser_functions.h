/**
 * Created by Carlo Pinciroli <cpinciro@ulb.ac.be>
 */
#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include <argos3/plugins/simulator/entities/light_entity.h>

using namespace argos;

class QuarantineQTUserFunctions : public CQTOpenGLUserFunctions {

public:

    QuarantineQTUserFunctions();

    virtual ~QuarantineQTUserFunctions() {}

    void Draw(CLightEntity& c_entity);

};

#endif
