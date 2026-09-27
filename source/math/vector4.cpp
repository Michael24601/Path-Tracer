
#include "math/vector4.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include "math/constants.hpp"
#include "math/vector3.hpp"


namespace pathtracer{


    Vector4 Vector4::ORIGIN = Vector4(0.0, 0.0, 0.0, 0.0);
    

    Vector4::Vector4() {
        m_data[0] = m_data[1] = m_data[2] = m_data[3] = 0.0;
    }

    Vector4::Vector4(real x, real y, real z, real w) {
        m_data[0] = x;
        m_data[1] = y;
        m_data[2] = z;
        m_data[3] = w;
    }

    Vector4::Vector4(const Vector3& v, real w) {
        m_data[0] = v.x();
        m_data[1] = v.y();
        m_data[2] = v.z();
        m_data[3] = w;
    }

    Vector4::Vector4(real x) {
        m_data[0] = m_data[1] = m_data[2] = m_data[3] = x;
    }

    real Vector4::x() const {
        return m_data[0];
    }

    real Vector4::y() const {
        return m_data[1];
    }

    real Vector4::z() const {
        return m_data[2];
    }

    real Vector4::w() const {
        return m_data[3];
    }

    void Vector4::setX(real x) {
        m_data[0] = x;
    }

    void Vector4::setY(real y) {
        m_data[1] = y;
    }

    void Vector4::setZ(real z) {
        m_data[2] = z;
    }

    void Vector4::setW(real w) {
        m_data[3] = w;
    }

    // Access using brackets
    const real& Vector4::operator[](int index) const {

        assert(
            (index >= 0 && index <= 3) &&
            "Index out of bounds");

        return m_data[index];
    }

    // Setter using brackets
    real& Vector4::operator[](int index) {

        assert(
            (index >= 0 && index <= 3) &&
            "Index out of bounds");

        return m_data[index];
    }

    // Element-wise max
    Vector4 Vector4::max(real r) const {

        return Vector4(
            std::max(r, m_data[0]),
            std::max(r, m_data[1]),
            std::max(r, m_data[2]),
            std::max(r, m_data[3]));
    }

    // Element-wise min
    Vector4 Vector4::min(real r) const {

        return Vector4(
            std::min(r, m_data[0]),
            std::min(r, m_data[1]),
            std::min(r, m_data[2]),
            std::min(r, m_data[3]));
    }

    real Vector4::lengthSquared() const {

        return m_data[0] * m_data[0]
            + m_data[1] * m_data[1]
            + m_data[2] * m_data[2]
            + m_data[3] * m_data[3];
    }

    real Vector4::length() const {
        return sqrtReal(lengthSquared());
    }

    // Element-wise absolute value of the vector
    Vector4 Vector4::abs() const {

        return Vector4(
            std::abs(m_data[0]),
            std::abs(m_data[1]),
            std::abs(m_data[2]),
            std::abs(m_data[3]));
    }

    Vector4 Vector4::operator-() const {

        Vector4 result(
            -m_data[0],
            -m_data[1],
            -m_data[2],
            -m_data[3]);

        return result;
    }

    Vector4 Vector4::operator+(const Vector4& v) const {

        Vector4 result(
            m_data[0] + v.m_data[0],
            m_data[1] + v.m_data[1],
            m_data[2] + v.m_data[2],
            m_data[3] + v.m_data[3]);

        return result;
    }

    Vector4 Vector4::operator-(const Vector4& v) const {

        Vector4 result(
            m_data[0] - v.m_data[0],
            m_data[1] - v.m_data[1],
            m_data[2] - v.m_data[2],
            m_data[3] - v.m_data[3]);

        return result;
    }

    // Scalar product
    Vector4 Vector4::operator*(real s) const {

        return Vector4(
            m_data[0] * s,
            m_data[1] * s,
            m_data[2] * s,
            m_data[3] * s);
    }

    // Dot product
    real Vector4::dot(const Vector4& v) const {

        return m_data[0] * v.m_data[0]
            + m_data[1] * v.m_data[1]
            + m_data[2] * v.m_data[2]
            + m_data[3] * v.m_data[3];
    }

    // Normalizes the vector
    void Vector4::normalize() {

        real d = length();

        if(std::abs(d) > EPSILON){
            m_data[0] /= d;
            m_data[1] /= d;
            m_data[2] /= d;
            m_data[3] /= d;
        }
    }

    // Returns a normalized copy of the vector
    Vector4 Vector4::normalized() const {

        Vector4 result(*this);
        result.normalize();

        return result;
    }

}