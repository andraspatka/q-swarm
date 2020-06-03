#ifndef Q_STATE
#define Q_STATE

#include <string>
#include <utility>
#include <argos3/core/utility/datatypes/color.h>

using namespace std;
using namespace argos;
namespace ql {
    class State {
    private:
        string stateName;
        CColor ledColor;
        unsigned short index;
    public:
        static State IDLE;
        static State WANDER;
        static State FOLLOW;
        static State DIR_LEFT;
        static State DIR_RIGHT;
        static State SEARCH;

        State(string stateName, CColor ledColor, unsigned short index) : stateName(std::move(stateName)), ledColor(ledColor), index(index) {}
        State() : stateName("INVALID"), ledColor(CColor::BLACK), index(65535) {}

        const string& getName() const {
            return this->stateName;
        }

        CColor getLedColor() const {
            return this->ledColor;
        }

        unsigned short getIndex() const {
            return this->index;
        }

        bool operator==(const State& rhs) const {
            return this->getIndex() == rhs.getIndex();
        }

        bool operator!=(const State& rhs) const {
            return this->getIndex() != rhs.getIndex();
        }
    };
    State State::WANDER("WANDER", CColor::BLACK, 0);
    State State::FOLLOW("FOLLOW", CColor::YELLOW, 1);
    State State::DIR_LEFT("DIR_LEFT", CColor::WHITE, 2);
    State State::DIR_RIGHT("DIR_RIGHT", CColor::WHITE, 3);
    State State::IDLE("IDLE", CColor::RED, 4);
    State State::SEARCH("SEARCH", CColor::CYAN, 5);
}
#endif