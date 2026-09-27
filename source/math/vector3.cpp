
#include "math/vector3.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include "math/constants.hpp"


namespace pathtracer{
    

    Vector3 Vector3::ORIGIN = Vector3(0.0, 0.0, 0.0);


    Vector3::Vector3() {
        m_data[0] = m_data[1] = m_data[2] = 0.0;
    }

    Vector3::Vector3(real x, real y, real z) {
        m_data[0] = x;
        m_data[1] = y;
        m_data[2] = z;
    }

    Vector3::Vector3(real x) {
        m_data[0] = m_data[1] = m_data[2] = x;
    }

    real Vector3::x() const {
        return m_data[0];
    }

    real Vector3::y() const {
        return m_data[1];
    }

    real Vector3::z() const {
        return m_data[2];
    }

    void Vector3::setX(real x) {
        m_data[0] = x;
    }

    void Vector3::setY(real y) {
        m_data[1] = y;
    }

    void Vector3::setZ(real z) {
        m_data[2] = z;
    }

    // Access using brackets
    const real& Vector3::operator[](int index) const {

        assert(
            (index >= 0 && index <= 2) &&
            "Index out of bounds");

        return m_data[index];
    }

    // Setter using brackets
    real& Vector3::operator[](int index) {

        assert(
            (index >= 0 && index <= 2) &&
            "Index out of bounds");

        return m_data[index];
    }

    // Element-wise max
    Vector3 Vector3::max(real r) const {

        return Vector3(
            std::max(r, m_data[0]),
            std::max(r, m_data[1]),
            std::max(r, m_data[2]));
    }

    // Element-wise min
    Vector3 Vector3::min(real r) const {

        return Vector3(
            std::min(r, m_data[0]),
            std::min(r, m_data[1]),
            std::min(r, m_data[2]));
    }

    // Element-wise max
    Vector3 Vector3::max(const Vector3& v) const {

        return Vector3(
            std::max(v.m_data[0], m_data[0]),
            std::max(v.m_data[1], m_data[1]),
            std::max(v.m_data[2], m_data[2]));
    }

    // Element-wise min
    Vector3 Vector3::min(const Vector3& v) const {

        return Vector3(
            std::min(v.m_data[0], m_data[0]),
            std::min(v.m_data[1], m_data[1]),
            std::min(v.m_data[2], m_data[2]));
    }

    real Vector3::lengthSquared() const {

        return m_data[0] * m_data[0]
            + m_data[1] * m_data[1]
            + m_data[2] * m_data[2];
    }

    real Vector3::length() const {
        return sqrtReal(lengthSquared());
    }

    real Vector3::mean() const {
        return ONE_THIRD *
            (m_data[0] + m_data[1] + m_data[2]);
    }

    // Element-wise absolute value of the vector
    Vector3 Vector3::abs() const {

        return Vector3(
            std::abs(m_data[0]),
            std::abs(m_data[1]),
            std::abs(m_data[2]));
    }

    Vector3 Vector3::operator-() const {

        Vector3 result(
            -m_data[0],
            -m_data[1],
            -m_data[2]);

        return result;
    }

    Vector3 Vector3::operator+(const Vector3& v) const {

        Vector3 result(
            m_data[0] + v.m_data[0],
            m_data[1] + v.m_data[1],
            m_data[2] + v.m_data[2]);

        return result;
    }

    Vector3 Vector3::operator-(const Vector3& v) const {

        Vector3 result(
            m_data[0] - v.m_data[0],
            m_data[1] - v.m_data[1],
            m_data[2] - v.m_data[2]);

        return result;
    }

    // Scalar product
    Vector3 Vector3::operator*(real s) const {

        return Vector3(
            m_data[0] * s,
            m_data[1] * s,
            m_data[2] * s);
    }

    // Scalar division
    Vector3 Vector3::operator/(real s) const {

        return Vector3(
            m_data[0] / s,
            m_data[1] / s,
            m_data[2] / s);
    }

    // Element-wise product
    Vector3 Vector3::operator*(const Vector3& v) const {

        return Vector3(
            m_data[0] * v.m_data[0],
            m_data[1] * v.m_data[1],
            m_data[2] * v.m_data[2]);
    }

    // Element-wise division
    Vector3 Vector3::operator/(const Vector3& v) const {

        return Vector3(
            m_data[0] / v.m_data[0],
            m_data[1] / v.m_data[1],
            m_data[2] / v.m_data[2]);
    }

    // Dot product
    real Vector3::dot(const Vector3& v) const {

        return m_data[0] * v.m_data[0]
            + m_data[1] * v.m_data[1]
            + m_data[2] * v.m_data[2];
    }

    // cross product
    Vector3 Vector3::cross(const Vector3& v) const {

        Vector3 result(
            m_data[1] * v.m_data[2] -
                m_data[2] * v.m_data[1],

            m_data[2] * v.m_data[0] -
                m_data[0] * v.m_data[2],

            m_data[0] * v.m_data[1] -
                m_data[1] * v.m_data[0]
        );

        return result;
    }

    // Normalizes the vector
    void Vector3::normalize() {

        real d = length();

        if(std::abs(d) > EPSILON){
            m_data[0] /= d;
            m_data[1] /= d;
            m_data[2] /= d;
        }
    }

    // Returns a normalized copy of the vector
    Vector3 Vector3::normalized() const {

        Vector3 result(*this);
        result.normalize();

        return result;
    }

    Vector3 Vector3::elementWiseMinimum(const Vector3& v) const {

        return Vector3(
            std::min(m_data[0], v.m_data[0]),
            std::min(m_data[1], v.m_data[1]),
            std::min(m_data[2], v.m_data[2])
        );
    }

    Vector3 Vector3::elementWiseMaximum(const Vector3& v) const {

        return Vector3(
            std::max(m_data[0], v.m_data[0]),
            std::max(m_data[1], v.m_data[1]),
            std::max(m_data[2], v.m_data[2])
        );
    }

    real Vector3::minElement() const {

        return std::min(
            m_data[0],
            std::min(m_data[1], m_data[2]));
    }

    real Vector3::maxElement() const {

        return std::max(
            m_data[0],
            std::max(m_data[1], m_data[2]));
    }

    // Element-wise >= operator
    bool Vector3::operator>=(const Vector3& other) const {

        return m_data[0] >= other.m_data[0]
            && m_data[1] >= other.m_data[1]
            && m_data[2] >= other.m_data[2];
    }

    bool Vector3::operator<=(const Vector3& other) const {

        return m_data[0] <= other.m_data[0]
            && m_data[1] <= other.m_data[1]
            && m_data[2] <= other.m_data[2];
    }

    bool Vector3::operator!=(const Vector3& other) const {

        return m_data[0] != other.m_data[0]
            || m_data[1] != other.m_data[1]
            || m_data[2] != other.m_data[2];
    }

    real Vector3::luminance() const {

        return 0.2126 * x()
            + 0.7152 * y()
            + 0.0722 * z();
    }

    std::string Vector3::toString() const {

        return "Vector("
            + std::to_string(m_data[0])
            + " "
            + std::to_string(m_data[1])
            + " "
            + std::to_string(m_data[2])
            + ")";
    }

}