
#ifndef PATH_TRACER_VECTOR3_HPP
#define PATH_TRACER_VECTOR3_HPP

#include <string>
#include "config.hpp"

namespace pathtracer{

    class Vector3{

    private:

        real m_data[3];

    public:

        static Vector3 ORIGIN;

        Vector3();

        Vector3(real x, real y, real z);

        Vector3(real x);

        real x() const;

        real y() const;

        real z() const;

        void setX(real x);

        void setY(real y);

        void setZ(real z);

        // Access using brackets
        const real& operator[](int index) const;

        // Setter using brackets
        real& operator[](int index);

        // Element-wise max
        Vector3 max(real r) const;

        // Element-wise min
        Vector3 min(real r) const;

        // Element-wise max
        Vector3 max(const Vector3& v) const;

        // Element-wise min
        Vector3 min(const Vector3& v) const;

        real lengthSquared() const;

        real length() const;

        real mean() const;

        // Element-wise absolute value of the vector
        Vector3 abs() const;

        Vector3 operator-() const;

        Vector3 operator+(const Vector3& v) const;

        Vector3 operator-(const Vector3& v) const;

        // Scalar product
        Vector3 operator*(real s) const;

        // Scalar division
        Vector3 operator/(real s) const;

        // Element-wise product
        Vector3 operator*(const Vector3& v) const;

        // Element-wise division
        Vector3 operator/(const Vector3& v) const;

        // Dot product
        real dot(const Vector3& v) const;

        // cross product
        Vector3 cross(const Vector3& v) const;

        // Normalizes the vector
        void normalize();

        // Returns a normalized copy of the vector
        Vector3 normalized() const;

        Vector3 elementWiseMinimum(const Vector3& v) const;

        Vector3 elementWiseMaximum(const Vector3& v) const;

        real minElement() const;

        real maxElement() const;

        // Element-wise >= operator
        bool operator>=(const Vector3& other) const;

        bool operator<=(const Vector3& other) const;

        bool operator!=(const Vector3& other) const;

        bool isFinite() const;

        real luminance() const;

        std::string toString() const;

    };

}

#endif