#ifndef Q_FOLLOWER_ACTION
#define Q_FOLLOWER_ACTION

#include "action.hpp"

namespace ql {

    class HighLevelAction : public Action {
    public:
        HighLevelAction(std::string actionName, unsigned short index) : Action(actionName, index) {}
        HighLevelAction() : Action("INVALID", 65535) {}

        static HighLevelAction FOLLOW;
        static HighLevelAction WANDER;
        static HighLevelAction STAY;
        static HighLevelAction AVOID;

        static const HighLevelAction& fromIndex(unsigned short index) {
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
    HighLevelAction HighLevelAction::FOLLOW("FOLLOW", 0);
    HighLevelAction HighLevelAction::WANDER("WANDER", 1);
    HighLevelAction HighLevelAction::STAY("STAY", 2);
    HighLevelAction HighLevelAction::AVOID("AVOID", 3);

}

#endif