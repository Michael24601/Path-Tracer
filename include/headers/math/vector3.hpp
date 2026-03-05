
#ifndef PATH_TRACER_VECTOR3_HPP
#define PATH_TRACER_VECTOR3_HPP

#include "../config.hpp"
#include "constants.hpp"

namespace pathtracer{

    class Vector3{

    private:

        real m_data[3];
        
    public:

        static Vector3 ORIGIN;    

        Vector3(){ m_data[0] = m_data[1] = m_data[2] = 0.0; }


        Vector3(real x, real y, real z){
            m_data[0] = x;
            m_data[1] = y;
            m_data[2] = z;
        }


        Vector3(real x){
            m_data[0] = m_data[1] = m_data[2] = x;
        }


        real x() const{ return m_data[0]; }


        real y() const{ return m_data[1]; }


        real z() const{ return m_data[2]; }


        void setX(real x){ m_data[0] = x; }


        void setY(real y){ m_data[1] = y; }


        void setZ(real z){ m_data[2] = z; }


        // Access using brackets
        const real& operator[](int index) const {
            assert((index >= 0 && index <= 2) && "Index out of bounds");
            return m_data[index];
        }


        // Setter using brackets
        real& operator[](int index) {
            assert((index >= 0 && index <= 2) && "Index out of bounds");
            return m_data[index];
        }


        // Element-wise max
        Vector3 max(real r) const {
            return Vector3(std::max(r, m_data[0]), std::max(r, m_data[1]), 
                std::max(r, m_data[2]));
        }


        // Element-wise min
        Vector3 min(real r) const {
            return Vector3(std::min(r, m_data[0]), std::min(r, m_data[1]), 
                std::min(r, m_data[2]));
        }


        // Element-wise max
        Vector3 max(const Vector3& v) const {
            return Vector3(std::max(v.m_data[0], m_data[0]), 
                std::max(v.m_data[1], m_data[1]), 
                std::max(v.m_data[2], m_data[2]));
        }


        // Element-wise min
        Vector3 min(const Vector3& v) const {
            return Vector3(std::min(v.m_data[0], m_data[0]), 
                std::min(v.m_data[1], m_data[1]), 
                std::min(v.m_data[2], m_data[2]));
        }


        real lengthSquared() const {
            return m_data[0] * m_data[0] 
                + m_data[1] * m_data[1] 
                + m_data[2] * m_data[2];
        }


        real length() const {
            return sqrtReal(lengthSquared());
        }


        // Element-wise absolute value of the vector
        Vector3 abs() const {
            return Vector3(std::abs(m_data[0]), std::abs(m_data[1]), 
                std::abs(m_data[2]));
        }

        
        Vector3 operator-() const{
            Vector3 result(-m_data[0], -m_data[1], -m_data[2]);
            return result;
        }


        Vector3 operator+(const Vector3& v) const{
            Vector3 result(m_data[0] + v.m_data[0], 
                m_data[1] + v.m_data[1], 
                m_data[2] + v.m_data[2]);
            return result;
        }


        Vector3 operator-(const Vector3& v) const{
            Vector3 result(m_data[0] - v.m_data[0], 
                m_data[1] - v.m_data[1], 
                m_data[2] - v.m_data[2]);
            return result;
        }


        // Scalar product
        Vector3 operator*(real s) const{
            return Vector3(m_data[0] * s, m_data[1] * s, m_data[2] * s);
        }


        // Element-wise product
        Vector3 operator*(const Vector3& v) const{
            return Vector3(m_data[0] * v.m_data[0], 
                m_data[1] * v.m_data[1], m_data[2] * v.m_data[2]);
        }



        // Dot product
        real dot(const Vector3& v) const{
            return m_data[0] * v.m_data[0]
                + m_data[1] * v.m_data[1]
                + m_data[2] * v.m_data[2];
        }


        // cross product
        Vector3 cross(const Vector3& v) const{
            Vector3 result(
                m_data[1] * v.m_data[2] - m_data[2] * v.m_data[1],
                m_data[2] * v.m_data[0] - m_data[0] * v.m_data[2],
                m_data[0] * v.m_data[1] - m_data[1] * v.m_data[0] 
            );
            return result;
        }


        // Normalizes the vector
        void normalize(){
            real d = length();
            if(std::abs(d) > EPSILON){
                m_data[0] /= d;
                m_data[1] /= d;
                m_data[2] /= d;
            }
        }


        // Returns a normalized copy of the vector
        Vector3 normalized() const{
            Vector3 result(*this);
            result.normalize();
            return result;
        }


         // Element-wise >= operator
        bool operator>=(const Vector3& other) const {
            return m_data[0] >= other.m_data[0] 
                && m_data[1] >= other.m_data[1]
                && m_data[2] >= other.m_data[2];
        }
        

        bool operator<=(const Vector3& other) const {
            return m_data[0] <= other.m_data[0] 
                && m_data[1] <= other.m_data[1]
                && m_data[2] <= other.m_data[2];
        }

    };

}

#endif