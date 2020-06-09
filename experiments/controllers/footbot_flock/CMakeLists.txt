add_library(footbot_flock MODULE
        footbot_flock.cpp
        footbot_flock.h)
target_link_libraries(
        footbot_flock
        qlearn
        potnavi
        monitoring
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
