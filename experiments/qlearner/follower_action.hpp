#ifndef Q_FOLLOWER_ACTION
#define Q_FOLLOWER_ACTION

#include "action.hpp"

namespace ql {

    class FollowerAction : public Action {
    public:
        FollowerAction(std::string actionName, unsigned short index) : Action(actionName, index, {0,0}) {}
        FollowerAction() : Action("INVALID", 65535, {0,0}) {}

        static FollowerAction FOLLOW;
        static FollowerAction WANDER;
        static FollowerAction STAY;
        static FollowerAction AVOID;

        static const FollowerAction& fromIndex(unsigned short index) {
            switch (index) {
                case 0:
                    return FOLLOW;
                case 1:
                    return WANDER;
                case 2:
                    return STAY;
                case 3:
                    return AVOID;
                default:
                    return STAY;
            }
        }

    };
    FollowerAction FollowerAction::FOLLOW("FOLLOW", 0);
    FollowerAction FollowerAction::WANDER("WANDER", 1);
    FollowerAction FollowerAction::STAY("STAY", 2);
    FollowerAction FollowerAction::AVOID("AVOID", 3);

}

#endif