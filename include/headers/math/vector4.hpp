
#ifndef PATH_TRACER_VECTOR4_HPP
#define PATH_TRACER_VECTOR4_HPP

#include "../config.hpp"
#include "constants.hpp"

namespace pathtracer{

    class Vector4{

    private:

        real m_data[4];
        
    public:

        static Vector4 ORIGIN;    

        Vector4(){ m_data[0] = m_data[1] = m_data[2] = m_data[3] = 0.0; }


        Vector4(real x, real y, real z, real w){
            m_data[0] = x;
            m_data[1] = y;
            m_data[2] = z;
            m_data[3] = w;
        }


        Vector4(real x){
            m_data[0] = m_data[1] = m_data[2] = m_data[3] = x;
        }


        real x() const{ return m_data[0]; }


        real y() const{ return m_data[1]; }


        real z() const{ return m_data[2]; }
        

        real w() const{ return m_data[3]; }


        void setX(real x){ m_data[0] = x; }


        void setY(real y){ m_data[1] = y; }


        void setZ(real z){ m_data[2] = z; }


        void setW(real w){ m_data[3] = w; }


        // Access using brackets
        const real& operator[](int index) const {
            assert((index >= 0 && index <= 3) && "Index out of bounds");
            return m_data[index];
        }


        // Setter using brackets
        real& operator[](int index) {
            assert((index >= 0 && index <= 3) && "Index out of bounds");
            return m_data[index];
        }


        // Element-wise max
        Vector4 max(real r) const {
            return Vector4(std::max(r, m_data[0]), std::max(r, m_data[1]), 
                std::max(r, m_data[2]), std::max(r, m_data[3]));
        }


        // Element-wise min
        Vector4 min(real r) const {
            return Vector4(std::min(r, m_data[0]), std::min(r, m_data[1]), 
                std::min(r, m_data[2]), std::min(r, m_data[3]));
        }


        real lengthSquared() const {
            return m_data[0] * m_data[0] + m_data[1] * m_data[1] 
                + m_data[2] * m_data[2] + m_data[3] * m_data[3];
        }


        real length() const {
            return sqrtReal(lengthSquared());
        }


        // Element-wise absolute value of the vector
        Vector4 abs() const {
            return Vector4(std::abs(m_data[0]), std::abs(m_data[1]), 
                std::abs(m_data[2]), std::abs(m_data[3]));
        }

        
        Vector4 operator-() const{
            Vector4 result(-m_data[0], -m_data[1], -m_data[2], -m_data[3]);
            return result;
        }


        Vector4 operator+(const Vector4& v) const{
            Vector4 result(m_data[0] + v.m_data[0], 
                m_data[1] + v.m_data[1], 
                m_data[2] + v.m_data[2],
                m_data[3] + v.m_data[3]);
            return result;
        }


        Vector4 operator-(const Vector4& v) const{
            Vector4 result(m_data[0] - v.m_data[0], 
                m_data[1] - v.m_data[1], 
                m_data[2] - v.m_data[2],
                m_data[3] - v.m_data[3]);
            return result;
        }


        // Scalar product
        Vector4 operator*(real s) const{
            return Vector4(m_data[0] * s, m_data[1] * s, 
                m_data[2] * s, m_data[3] * s);
        }


        // Dot product
        real dot(const Vector4& v) const{
            return m_data[0] * v.m_data[0]
                + m_data[1] * v.m_data[1]
                + m_data[2] * v.m_data[2]
                + m_data[3] * v.m_data[3];
        }


        // Normalizes the vector
        void normalize(){
            real d = length();
            if(std::abs(d) > EPSILON){
                m_data[0] /= d;
                m_data[1] /= d;
                m_data[2] /= d;
                m_data[3] /= d;
            }
        }


        // Returns a normalized copy of the vector
        Vector4 normalized() const{
            Vector4 result(*this);
            result.normalize();
            return result;
        }

    };

}

#endif