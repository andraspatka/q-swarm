add_library(footbot_follow MODULE footbot_follow.cpp footbot_follow.h ../../qlearner/thread_safe_random.hpp ../../qlearner/qlearner.hpp)
target_link_libraries(
        footbot_follow
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
