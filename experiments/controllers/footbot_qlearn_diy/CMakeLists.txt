add_library(footbot_qlearn_diy MODULE footbot_qlearn_diy.h footbot_qlearn_diy.cpp ../../qlearner/thread_safe_random.hpp ../../qlearner/qlearner.hpp)
target_link_libraries(
        footbot_qlearn_diy
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
