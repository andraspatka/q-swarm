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
}
#endif