#ifndef Q_VECTOR
#define Q_VECTOR

#include <argos3/core/utility/math/vector2.h>

using namespace argos;

namespace ql {
    class Vector {
        double length;
        double angle;
    public:
        Vector() : length(0.0f), angle(0.0f) {}

        Vector(const Vector& vector) : length(vector.getLength()), angle(vector.getAngle()) {}

        Vector(double length, double angle) : length(length), angle(angle) {}

        Vector(double length, CRadians angle) : length(length) {
            this->angle = angle.GetValue() * argos::CRadians::RADIANS_TO_DEGREES;
        }

        bool isZero() {
            return this->length <= 0.01;
        }

        void clampLength(const double minBound, const double maxBound) {
            if (this->length > maxBound) {
                this->length = maxBound;
            }
            if (this->length < minBound) {
                this->length = minBound;
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

        Vector& operator=(const Vector& rhs) {
            this->length = rhs.getLength();
            this->angle = rhs.getAngle();
            return *this;
        }

        Vector& operator+=(const Vector& rhs) {
            Vector v3 = operator+(rhs);
            this->length = v3.length;
            this->angle = v3.angle;
            return *this;
        }

        Vector operator+(const Vector& rhs) const{
            CVector2 v2;
            v2.FromPolarCoordinates(rhs.length, rhs.getAngleInRadians());

            CVector2 v1;
            v1.FromPolarCoordinates(this->length, this->getAngleInRadians());

            CVector2 v3 = v1 + v2;
            Vector ret = {v3.Length(), v3.Angle().GetValue() * argos::CRadians::RADIANS_TO_DEGREES};
            ret.length = ret.isZero() ? 0 : ret.length;
            return ret;
        }

        Vector operator-() const {
            double angle = this->angle;
            if (this->angle > 0) {
                angle = -180 + angle;
            } else {
                angle = 180 + angle;
            }
            return {this->length, angle};
        }

        Vector operator*(double factor) const {
            return {factor * this->length, this->angle};
        }

    };
}
#endif