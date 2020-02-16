add_library(footbot_manual SHARED footbot_manual.h footbot_manual.cpp)
target_link_libraries(
        footbot_manual
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
