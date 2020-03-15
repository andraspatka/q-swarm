add_library(footbot_follow MODULE
            footbot_follow.cpp
            footbot_follow.h)
target_link_libraries(
        footbot_follow
        qlearn
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
