
#ifndef PATH_TRACER_VECTOR2_HPP
#define PATH_TRACER_VECTOR2_HPP

#include <string>
#include "config.hpp"

namespace pathtracer{

    class Vector2{

    private:

        real m_data[2];

    public:

        static Vector2 ORIGIN;

        Vector2();

        Vector2(real x, real y);

        Vector2(real x);

        real x() const;

        real y() const;

        // Access using brackets
        const real& operator[](int index) const;

        // Setter using brackets
        real& operator[](int index);

        real lengthSquared() const;

        real length() const;

        // Element-wise absolute value of the vector
        Vector2 abs() const;

        Vector2 operator-() const;

        Vector2 operator+(const Vector2& v) const;

        Vector2 operator-(const Vector2& v) const;

        Vector2 operator/(real s) const;

        // Scalar product
        Vector2 operator*(real s) const;

        // Dot product
        real dot(const Vector2& v) const;

        // Normalizes the vector
        void normalize();

        // Returns a normalized copy of the vector
        Vector2 normalized() const;

        // Scalar product
        bool operator<(const Vector2& v) const;

        std::string toString() const;

    };

}

#endif