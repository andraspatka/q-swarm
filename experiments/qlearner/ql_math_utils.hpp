#ifndef QL_MATH_UTILS
#define QL_MATH_UTILS

#include <array>
#include <cassert>
#include <cmath>

#define assertm(exp, msg) assert(((void)msg, exp))

namespace ql {

    class QLMathUtils {
    public:
        static constexpr double EPSILON = 0.01;
        static bool closeToZero(double value) {
            return value < EPSILON && value > -EPSILON;
        }

        /**
         *
         * @param a height of the gauss curve
         * @param x distance
         * @param b centre point of the gauss curve
         * @param c width of the curve
         * @return
         */
        static double calculateGauss(double a, double x, double b, double c) {
            double num = - pow((x - b), 2.0);
            double denum = 2 * pow(c, 2.0);
            return a * exp(num / denum);
        }
    };
}
#endif



