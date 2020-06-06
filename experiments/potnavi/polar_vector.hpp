#ifndef Q_VECTOR
#define Q_VECTOR

#include <argos3/core/utility/math/vector2.h>

using namespace argos;

namespace ql {
    class PolarVector {
        double length;
        double angle;
    public:
        PolarVector() : length(0.0f), angle(0.0f) {}

        PolarVector(const PolarVector& vector) : length(vector.getLength()), angle(vector.getAngle()) {}

        PolarVector(double length, double angle) : length(length), angle(angle) {}

        PolarVector(double length, CRadians angle) : length(length) {
            this->angle = angle.GetValue() * argos::CRadians::RADIANS_TO_DEGREES;
        }

        bool isZero() {
            return this->length <= 0.05;
        }

        void clampZeroAndMax(const double maxBound) {
            if (this->length > maxBound) {
                this->length = maxBound;
            }
            if (this->isZero()) {
                this->length = 0;
            }
        }

        double getLength() const {
            return this->length;
        }

        double getAbsAngle() {
            if (this->angle < 0) {
                return -this->angle;
            }
            return this->angle;
        }

        CRadians getAngleInRadians() const {
            return CRadians(this->angle / argos::CRadians::RADIANS_TO_DEGREES);
        }

        double getAngle() const {
            return this->angle;
        }

        PolarVector& operator=(const PolarVector& rhs) {
            this->length = rhs.getLength();
            this->angle = rhs.getAngle();
            return *this;
        }

        PolarVector& operator+=(const PolarVector& rhs) {
            PolarVector v3 = operator+(rhs);
            this->length = v3.length;
            this->angle = v3.angle;
            return *this;
        }

        PolarVector operator+(const PolarVector& rhs) const{
            CVector2 v2;
            v2.FromPolarCoordinates(rhs.length, rhs.getAngleInRadians());

            CVector2 v1;
            v1.FromPolarCoordinates(this->length, this->getAngleInRadians());

            CVector2 v3 = v1 + v2;
            PolarVector ret = {v3.Length(), v3.Angle().GetValue() * argos::CRadians::RADIANS_TO_DEGREES};
            ret.length = ret.isZero() ? 0 : ret.length;
            return ret;
        }

        PolarVector operator-() const {
            double angle = this->angle;
            if (this->angle > 0) {
                angle = -180 + angle;
            } else {
                angle = 180 + angle;
            }
            return {this->length, angle};
        }

        PolarVector operator*(double factor) const {
            return {factor * this->length, this->angle};
        }

    };
}
#endif