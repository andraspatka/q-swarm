add_library(footbot_leader MODULE
        footbot_leader.h
        footbot_leader.cpp)
target_link_libraries(
        footbot_leader
        qlearn
        potnavi
        monitoring
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
