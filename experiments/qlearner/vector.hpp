#ifndef Q_VECTOR
#define Q_VECTOR

#include <argos3/core/utility/math/vector2.h>

using namespace argos;

namespace ql {
    class Vector : public CVector2 {
    public:
        Vector() : CVector2() {}

        Vector(const CVector2& vector) : CVector2(vector.Length(), vector.Angle()) {}

        Vector(double length, CRadians angle) : CVector2(length, angle) {}

        bool isZero() {
            return this->Length() < 0.001 && this->Length() > -0.001;
        }

        void clampLength(const double minBound, const double maxBound) {
            double vectorLength = this->Length();
            if (vectorLength > maxBound) {
                vectorLength = 1;
            }
            if (vectorLength < minBound) {
                vectorLength = 0;
            }
            CVector2 vector = {vectorLength, this->Angle()};
            *this = vector;
        }

        double getAbsAngle() {
            return this->Angle().GetAbsoluteValue() * argos::CRadians::RADIANS_TO_DEGREES;
        }

        double getAngle() {
            return this->Angle().GetValue() * argos::CRadians::RADIANS_TO_DEGREES;
        }

    };
}
#endif