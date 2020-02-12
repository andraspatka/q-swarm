add_library(footbot_qlearn_diy MODULE footbot_qlearn_diy.h footbot_qlearn_diy.cpp thread_safe_random.hpp)
target_link_libraries(
        footbot_qlearn_diy
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
