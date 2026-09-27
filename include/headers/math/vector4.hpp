
#ifndef PATH_TRACER_VECTOR4_HPP
#define PATH_TRACER_VECTOR4_HPP

#include "config.hpp"

namespace pathtracer{

    // Forward declarations
    class Vector3;

    class Vector4{

    private:

        real m_data[4];

    public:

        static Vector4 ORIGIN;

        Vector4();

        Vector4(real x, real y, real z, real w);

        Vector4(const Vector3& v, real w);

        Vector4(real x);

        real x() const;

        real y() const;

        real z() const;

        real w() const;

        void setX(real x);

        void setY(real y);

        void setZ(real z);

        void setW(real w);

        // Access using brackets
        const real& operator[](int index) const;

        // Setter using brackets
        real& operator[](int index);

        // Element-wise max
        Vector4 max(real r) const;

        // Element-wise min
        Vector4 min(real r) const;

        real lengthSquared() const;

        real length() const;

        // Element-wise absolute value of the vector
        Vector4 abs() const;

        Vector4 operator-() const;

        Vector4 operator+(const Vector4& v) const;

        Vector4 operator-(const Vector4& v) const;

        // Scalar product
        Vector4 operator*(real s) const;

        // Dot product
        real dot(const Vector4& v) const;

        // Normalizes the vector
        void normalize();

        // Returns a normalized copy of the vector
        Vector4 normalized() const;

    };

}

#endif