#ifndef QL_MATH_UTILS
#define QL_MATH_UTILS

#include <array>
#include <cassert>
#include <cmath>
#include <argos3/core/utility/math/vector2.h>

#define assertm(exp, msg) assert(((void)msg, exp))

namespace ql {

    class QLMathUtils {
    public:
        static constexpr double EPSILON = 0.01;
        static constexpr double CUTOFF_VALUE = 0.15;
        static constexpr double CAMERA_READING_MAX_VALUE = 120.0f;

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
            double num = -pow((x - b), 2.0);
            double denum = 2 * pow(c, 2.0);
            double ret = a * exp(num / denum);
            return ret <= CUTOFF_VALUE ? 0.0f : ret;
        }

        static double cameraToDistance(double reading) {
            return reading / CAMERA_READING_MAX_VALUE;
        }

        static double proxToDistance(double reading) {
            return 1 - reading;
        }

        static argos::CVector2
        readingToVector(double readingLength, argos::CRadians readingAngle, double a, double b, double c,
                        double (*transf)(double)) {
            double length = calculateGauss(a, transf(readingLength), b, c);
            return {length, readingAngle};
        }

        static double absAngleInDegrees(argos::CRadians angle) {
            return angle.GetAbsoluteValue() * argos::CRadians::RADIANS_TO_DEGREES;
        }

        static double angleInDegrees(argos::CRadians angle) {
            return angle.GetValue() * argos::CRadians::RADIANS_TO_DEGREES;
        }
    };
}
#endif



