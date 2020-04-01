add_library(footbot_leader MODULE footbot_leader.h footbot_leader.cpp ../../qlearner/thread_safe_random.hpp ../../qlearner/qlearner.hpp)
target_link_libraries(
        footbot_leader
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
