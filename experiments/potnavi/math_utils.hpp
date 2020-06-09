#ifndef MATH_UTILS
#define MATH_UTILS

#include <array>
#include <cassert>
#include <cmath>
#include <argos3/core/utility/math/vector2.h>
#include "polar_vector.hpp"

#define assertm(exp, msg) assert(((void)msg, exp))

using namespace argos;

namespace ql {

    class MathUtils {
    public:
        static constexpr double EPSILON = 0.001;
        static constexpr double CUTOFF_VALUE = 0.05;
        static constexpr double CAMERA_READING_MAX_VALUE = 95.0f;

        static constexpr double WHEEL_RADIUS = 0.029112741f;
        static constexpr double INTERWHEEL_DISTANCE = 0.14f;

        static constexpr double C = (WHEEL_RADIUS / 2) * (1 + 1 / (INTERWHEEL_DISTANCE / 2) ); // 0.22

        static constexpr double PI_OVER_TWO = 1.57079632679;

        static bool closeToZero(double value) {
            return value < EPSILON && value > -EPSILON;
        }

        static double clampValue(double value, double minBound, double maxBound) {
            if (value < minBound) {
                return minBound;
            }
            if (value > maxBound) {
                return maxBound;
            }
            return value;
        }

        static std::array<double, 2> vectorToLinearVelocity(const PolarVector& vector) {
            double v = vector.getLength() * PI_OVER_TWO;
            double w = -clampValue(vector.getAngleInRadians().GetValue(), -PI_OVER_TWO, PI_OVER_TWO);

            double phiLeft = (v + w) / C;
            double phiRight = (v - w) / C;

            return{ phiLeft * 10, phiRight * 10};
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
            return (ret <= CUTOFF_VALUE) ? 0.0f : ret;
        }

        static double cameraToDistance(double reading) {
            return (reading / CAMERA_READING_MAX_VALUE) * 3;
        }

        static double proxToDistance(double reading) {
            return 1 - reading;
        }

        static double lightToDistance(double reading) {
            return (1 - reading) * 2.3;
        }

        static ql::PolarVector
        readingToVector(double readingLength, argos::CRadians readingAngle, double a, double b, double c,
                        double (*transf)(double)) {
            double length = calculateGauss(a, transf(readingLength), b, c);
            return {length, readingAngle};
        }
    };
}
#endif



