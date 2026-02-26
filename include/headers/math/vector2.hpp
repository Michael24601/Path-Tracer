
#ifndef PATH_TRACER_VECTOR2_HPP
#define PATH_TRACER_VECTOR2_HPP

#include "../config.hpp"
#include "constants.hpp"

namespace pathtracer{

    class Vector2{

    private:

        real m_data[2];
        
    public: 

        static Vector2 ORIGIN;    

        
        Vector2(){ m_data[0] = m_data[1] = 0.0; }


        Vector2(real x, real y){
            m_data[0] = x;
            m_data[1] = y;
        }


        Vector2(real x){
            m_data[0] = m_data[1] = x;
        }


        real x() const{ return m_data[0]; }


        real y() const{ return m_data[1]; }


        // Access using brackets
        const real& operator[](int index) const {
            assert((index >= 0 && index <= 1) && "Index out of bounds");
            return m_data[index];
        }


        // Setter using brackets
        real& operator[](int index) {
            assert((index >= 0 && index <= 1) && "Index out of bounds");
            return m_data[index];
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
        Vector2 abs() const {
            return Vector2(std::abs(m_data[0]), std::abs(m_data[1]));
        }


        Vector2 operator-() const{
            Vector2 result(-m_data[0], -m_data[1]);
            return result;
        }


        Vector2 operator+(const Vector2& v) const{
            Vector2 result(m_data[0] + v.m_data[0], 
                m_data[1] + v.m_data[1]);
            return result;
        }


        Vector2 operator-(const Vector2& v) const{
            Vector2 result(m_data[0] - v.m_data[0], 
                m_data[1] - v.m_data[1]);
            return result;
        }


        // Scalar product
        Vector2 operator*(real s) const{
            return Vector2(m_data[0] * s, m_data[1] * s);
        }


        // Dot product
        real dot(const Vector2& v) const{
            return m_data[0] * v.m_data[0]
                + m_data[1] * v.m_data[1];
        }


        // Normalizes the vector
        void normalize(){
            real d = length();
            if(std::abs(d) > EPSILON){
                m_data[0] /= d;
                m_data[1] /= d;
            }
        }


        // Returns a normalized copy of the vector
        Vector2 normalized() const{
            Vector2 result(*this);
            result.normalize();
            return result;
        }


    };

}

#endif