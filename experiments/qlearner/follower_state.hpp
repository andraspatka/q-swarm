#ifndef Q_FOLLOWER_STATE
#define Q_FOLLOWER_STATE

#include "state.hpp"


using namespace std;
using namespace argos;
namespace ql {
    class FollowerState : public State {
    public:
        static FollowerState GOAL_REACHED;
        static FollowerState LEADER_DETECTED;
        static FollowerState INDIRECT_LEADER_DETECTED;
        static FollowerState OBSTACLE_DETECTED;
        static FollowerState UNKNOWN;

        FollowerState(string stateName, CColor ledColor, unsigned short index) : State(stateName, ledColor, index) {}
        FollowerState() : State("INVALID", CColor::BLACK, 65535) {}
    };
    FollowerState FollowerState::GOAL_REACHED("GOAL_REACHED", CColor::YELLOW, 0);
    FollowerState FollowerState::LEADER_DETECTED("LEADER_DETECTED", CColor::YELLOW, 1);
    FollowerState FollowerState::INDIRECT_LEADER_DETECTED("INDIRECT_LEADER_DETECTED", CColor::CYAN, 2);
    FollowerState FollowerState::OBSTACLE_DETECTED("OBSTACLE_DETECTED", CColor::WHITE, 3);
    FollowerState FollowerState::UNKNOWN("UNKNOWN", CColor::WHITE, 4);
}
#endif