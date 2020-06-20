#ifndef Q_FLOCK_ACTION
#define Q_FLOCK_ACTION

#include <string>
#include <array>
#include "action.hpp"

namespace ql {
    class LowLevelAction : public Action {
    public:
        LowLevelAction(std::string actionName, unsigned short index) : Action(actionName, index) {}

        static LowLevelAction STOP;
        static LowLevelAction FORWARD;
        static LowLevelAction TURN_LEFT;
        static LowLevelAction TURN_RIGHT;

        static const LowLevelAction& fromIndex(unsigned short index) {
            switch (index) {
                case 0:
                    return STOP;
                case 1:
                    return TURN_LEFT;
                case 2:
                    return TURN_RIGHT;
                case 3:
                    return FORWARD;
                default:
                    return STOP;
            }
        }
    };

    LowLevelAction LowLevelAction::STOP("STOP", 0);
    LowLevelAction LowLevelAction::TURN_LEFT("TURN_LEFT", 1);
    LowLevelAction LowLevelAction::TURN_RIGHT("TURN_RIGHT", 2);
    LowLevelAction LowLevelAction::FORWARD("FORWARD", 3);
}
#endif