#ifndef QL_UTILS
#define QL_UTILS

#include <vector>
#include <fstream>
#include <iostream>
#include <tuple>
#include <cassert>

#define assertm(exp, msg) assert(((void)msg, exp))

namespace ql {

    class QLUtils {
    public:
        static std::string getActionName(double x, double y) {
            if (x == 0.0 && y == 0.0) return "STOP";
            if (x == y) return "FORWARD";
            if (y > x) return "TURN LEFT";
            if (x > y) return "TURN RIGHT";
            return "INVALID";
        }

        static std::array<double, 2> getActionFromIndex(int actionIndex, double velocity) {
            std::array<double, 2> action{};
            switch (actionIndex) {
                case 0: // STOP
                    action[0] = 0;
                    action[1] = 0;
                    break;
                case 1: // TURN LEFT
                    action[0] = 0;
                    action[1] = 2 * velocity;
                    break;
                case 2: // TURN RIGHT
                    action[0] = 2 * velocity;
                    action[1] = 0;
                    break;
                case 3: // FORWARD
                    action[0] = velocity;
                    action[1] = velocity;
                    break;
                default:
                    action[0] = 0;
                    action[1] = 0;
                    break;
            }
            return action;
        }
    };
}
#endif



