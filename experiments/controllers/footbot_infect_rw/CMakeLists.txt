add_library(footbot_infect_rw MODULE
        footbot_infect_rw.cpp
        footbot_infect_rw.h)
target_link_libraries(
        footbot_infect_rw
        qlearn
        monitoring
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
