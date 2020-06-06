#ifndef Q_FOLLOWER_ACTION
#define Q_FOLLOWER_ACTION

#include "action.hpp"

namespace ql {

    class FollowerAction : public Action {
    public:
        FollowerAction(std::string actionName, unsigned short index) : Action(actionName, index, {0,0}) {}
        FollowerAction() : Action("INVALID", 65535, {0,0}) {}

        static FollowerAction FOLLOW_LEADER;
        static FollowerAction FOLLOW_INDIRECT_LEADER;
        static FollowerAction WANDER;
        static FollowerAction STAY;
        static FollowerAction AVOID;

    };
    FollowerAction FollowerAction::FOLLOW_LEADER("FOLLOW_LEADER", 0);
    FollowerAction FollowerAction::FOLLOW_INDIRECT_LEADER("FOLLOW_INDIRECT_LEADER", 1);
    FollowerAction FollowerAction::WANDER("WANDER", 2);
    FollowerAction FollowerAction::STAY("STAY", 3);
    FollowerAction FollowerAction::AVOID("AVOID", 4);

}

#endif