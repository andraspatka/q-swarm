add_library(qlearn MODULE
        qlearner.hpp
        qexploiter.hpp
        state/state.hpp
        state/simple_state.hpp
        state/follower_state.hpp
        agent_type.hpp
        action/action.hpp
        action/low_level_action.hpp
        action/high_level_action.hpp
        stage.hpp
        thread_safe_random.hpp)