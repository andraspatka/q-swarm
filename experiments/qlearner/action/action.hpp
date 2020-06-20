#ifndef Q_ACTION
#define Q_ACTION

#include <string>
#include <array>
#include <utility>

namespace ql {
    class Action {
    private:
        std::string actionName;
        unsigned short index;
    public:

        static Action INVALID_ACTION;
        Action(std::string actionName, unsigned short index) : actionName(std::move(actionName)), index(index) {}

        const std::string& getName() const {
            return actionName;
        }

        unsigned short getIndex() const {
            return index;
        }

        static const Action& fromIndex(unsigned short index) {
            return INVALID_ACTION;
        }

        bool operator==(const Action& rhs) const {
            return this->getIndex() == rhs.getIndex();
        }
    };
    Action Action::INVALID_ACTION("INVALID", 0);
}
#endif