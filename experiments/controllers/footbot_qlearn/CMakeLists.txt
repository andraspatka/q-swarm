add_library(footbot_qlearn MODULE footbot_qlearn.h footbot_qlearn.cpp)
target_link_libraries(
        footbot_qlearn
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        /usr/local/lib/fido.a)
