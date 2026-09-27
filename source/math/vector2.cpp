
#include "math/vector2.hpp"
#include <cassert>
#include <cmath>
#include "math/constants.hpp"

namespace pathtracer{


    Vector2 Vector2::ORIGIN = Vector2(0.0, 0.0);

    
    Vector2::Vector2() {
        m_data[0] = m_data[1] = 0.0;
    }

    Vector2::Vector2(real x, real y) {
        m_data[0] = x;
        m_data[1] = y;
    }

    Vector2::Vector2(real x) {
        m_data[0] = m_data[1] = x;
    }

    real Vector2::x() const {
        return m_data[0];
    }

    real Vector2::y() const {
        return m_data[1];
    }

    // Access using brackets
    const real& Vector2::operator[](int index) const {

        assert(
            (index >= 0 && index <= 1) &&
            "Index out of bounds");

        return m_data[index];
    }

    // Setter using brackets
    real& Vector2::operator[](int index) {

        assert(
            (index >= 0 && index <= 1) &&
            "Index out of bounds");

        return m_data[index];
    }

    real Vector2::lengthSquared() const {

        return m_data[0] * m_data[0]
            + m_data[1] * m_data[1];
    }

    real Vector2::length() const {
        return sqrtReal(lengthSquared());
    }

    // Element-wise absolute value of the vector
    Vector2 Vector2::abs() const {

        return Vector2(
            std::abs(m_data[0]),
            std::abs(m_data[1]));
    }

    Vector2 Vector2::operator-() const {

        Vector2 result(
            -m_data[0],
            -m_data[1]);

        return result;
    }

    Vector2 Vector2::operator+(const Vector2& v) const {

        Vector2 result(
            m_data[0] + v.m_data[0],
            m_data[1] + v.m_data[1]);

        return result;
    }

    Vector2 Vector2::operator-(const Vector2& v) const {

        Vector2 result(
            m_data[0] - v.m_data[0],
            m_data[1] - v.m_data[1]);

        return result;
    }

    Vector2 Vector2::operator/(real s) const {

        return Vector2(
            m_data[0] / s,
            m_data[1] / s);
    }

    // Scalar product
    Vector2 Vector2::operator*(real s) const {

        return Vector2(
            m_data[0] * s,
            m_data[1] * s);
    }

    // Dot product
    real Vector2::dot(const Vector2& v) const {

        return m_data[0] * v.m_data[0]
            + m_data[1] * v.m_data[1];
    }

    // Normalizes the vector
    void Vector2::normalize() {

        real d = length();

        if(std::abs(d) > EPSILON){
            m_data[0] /= d;
            m_data[1] /= d;
        }
    }

    // Returns a normalized copy of the vector
    Vector2 Vector2::normalized() const {

        Vector2 result(*this);
        result.normalize();

        return result;
    }

    // Scalar product
    bool Vector2::operator<(const Vector2& v) const {

        return m_data[0] < v.m_data[0] &&
            m_data[1] < v.m_data[1];
    }

    std::string Vector2::toString() const {

        return "Vector("
            + std::to_string(m_data[0])
            + " "
            + std::to_string(m_data[1])
            + ")";
    }

}