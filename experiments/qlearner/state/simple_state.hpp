#ifndef Q_SIMPLE_STATE
#define Q_SIMPLE_STATE

#include "state.hpp"

using namespace std;
using namespace argos;

namespace ql {
    class SimpleState : public State {
    public:

        static SimpleState IDLE;
        static SimpleState WANDER;
        static SimpleState FOLLOW;
        static SimpleState DIR_LEFT;
        static SimpleState DIR_RIGHT;

        SimpleState(string stateName, CColor ledColor, unsigned short index) : State(stateName, ledColor, index) {}
        SimpleState() : State("INVALID", CColor::BLACK, 65535) {}
    };
    SimpleState SimpleState::WANDER("WANDER", CColor::BLACK, 0);
    SimpleState SimpleState::FOLLOW("FOLLOW", CColor::YELLOW, 1);
    SimpleState SimpleState::DIR_LEFT("DIR_LEFT", CColor::WHITE, 2);
    SimpleState SimpleState::DIR_RIGHT("DIR_RIGHT", CColor::WHITE, 3);
    SimpleState SimpleState::IDLE("IDLE", CColor::RED, 4);
}
#endif