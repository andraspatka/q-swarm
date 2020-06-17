#ifndef Q_FOLLOWER_STATE
#define Q_FOLLOWER_STATE

#include "state.hpp"


using namespace std;
using namespace argos;
namespace ql {
    class FollowerState : public State {
    public:

        static FollowerState NO_TARGET_TO_FOLLOW;
        static FollowerState TARGET_FOLLOW;
        static FollowerState TARGET_REACHED;
        static FollowerState OBSTACLE_DETECTED;
        static FollowerState OBSTACLE_DETECTED_TARGET_FRONT;
        static FollowerState OBSTACLE_DETECTED_TARGET_BACK;


        FollowerState(string stateName, CColor ledColor, unsigned short index) : State(stateName, ledColor, index) {}
        FollowerState() : State("INVALID", CColor::BLACK, 65535) {}
    };
    FollowerState FollowerState::NO_TARGET_TO_FOLLOW("NO_TARGET_TO_FOLLOW", CColor::BLACK, 0);
    FollowerState FollowerState::TARGET_FOLLOW("TARGET_FOLLOW", CColor::YELLOW, 1);
    FollowerState FollowerState::TARGET_REACHED("TARGET_REACHED", CColor::PURPLE, 2);
    FollowerState FollowerState::OBSTACLE_DETECTED("OBSTACLE_DETECTED", CColor::WHITE, 3);
    FollowerState FollowerState::OBSTACLE_DETECTED_TARGET_FRONT("OBSTACLE_DETECTED_TARGET_FRONT", CColor::YELLOW, 4);
    FollowerState FollowerState::OBSTACLE_DETECTED_TARGET_BACK("OBSTACLE_DETECTED_TARGET_BACK", CColor::WHITE, 5);
}
#endif