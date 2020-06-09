add_library(footbot_snake MODULE
        footbot_snake.cpp
        footbot_snake.h)
target_link_libraries(
        footbot_snake
        qlearn
        potnavi
        monitoring
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
