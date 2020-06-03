#ifndef Q_ACTION
#define Q_ACTION

#include <string>
#include <array>

namespace ql {
    class Action {
    private:
        std::string actionName;
        unsigned short index;
        std::array<double, 2> wheelSpeed;
    public:
        Action(std::string actionName, unsigned short index, std::array<double, 2> wheelSpeed) : actionName(actionName),
                                                                                      index(index),
                                                                                      wheelSpeed(wheelSpeed) {}

        static Action STOP;
        static Action FORWARD;
        static Action TURN_LEFT;
        static Action TURN_LEFT_SOFT;
        static Action TURN_RIGHT;
        static Action TURN_RIGHT_SOFT;

        const std::string& getName() const {
            return actionName;
        }

        unsigned short getIndex() const {
            return index;
        }

        const std::array<double, 2>& getWheelSpeed() const {
            return wheelSpeed;
        }

        static const Action& fromIndex(unsigned short index) {
            switch(index) {
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

        bool operator==(const Action& rhs) const {
            return this->getIndex() == rhs.getIndex();
        }
    };

    Action Action::STOP("STOP", 0, {0.0f, 0.0f});
    Action Action::TURN_LEFT("TURN_LEFT", 1, {-0.5f, 0.5f});
    Action Action::TURN_RIGHT("TURN_RIGHT", 2, {0.5, -0.5f});
    Action Action::FORWARD("FORWARD", 3, {1.0f, 1.0f});
}
#endif