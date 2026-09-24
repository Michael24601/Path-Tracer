
#ifndef PATH_TRACER_MATRIX2_HPP
#define PATH_TRACER_MATRIX2_HPP

#include "../config.hpp"
#include "constants.hpp"
#include "mathUtil.hpp"

namespace pathtracer{

    class Matrix2{

    private:

        real m_data[4];
        
        
    public:

        static Matrix2 IDENTITY;    


        // Identity
        Matrix2(){ 
            m_data[1] = m_data[2] = 0.0; 
            m_data[0] = m_data[3] = 1.0;
        }


        // These are assumed to be columns
        Matrix2(real c0, real c1, real c2, real c3){
            m_data[0] = c0;
            m_data[1] = c1;
            m_data[2] = c2;
            m_data[3] = c3;
        }


        // These are assumed to be columns
        Matrix2(const Vector2& c0, const Vector2& c1){
            m_data[0] = c0.x();
            m_data[1] = c1.x();
            m_data[2] = c0.y();
            m_data[3] = c1.y();
        }


        static Matrix2 rotationMatrix(real angle) {

            real c = std::cos(angle);
            real s = std::sin(angle);
            return Matrix2(
                Vector2(c, s),
                Vector2(-s, c)
            );
        }

        // Column getter
        Vector2 column(int col) const{ 
            assert((col >= 0 && col < 2) && "Column out of bounds");
            return Vector2(m_data[col], m_data[col+2]);
        }


        void setColumn(const Vector2& v, int col){ 
            assert((col >= 0 && col < 2) && "Column out of bounds");
            m_data[col] = v.x();
            m_data[col+2] = v.y();
        }


        const real& operator()(int row, int col) const {
            assert((row >= 0 && row < 2) && (col >= 0 && col < 2)
                && "Index out of bounds");
            return m_data[row * 2 + col];
        }


        real& operator()(int row, int col) {
            assert((row >= 0 && row < 2) && (col >= 0 && col < 2)
                && "Index out of bounds");
            return m_data[row * 2 + col];
        }


        void transpose(){
            Util::swap(m_data[1], m_data[3]);
        }


        Matrix2 transposed() const{
            Matrix2 t = *this;
            t.transpose();
            return t;
        }

        real determinant() const {
            return m_data[0] * m_data[3] - m_data[1] * m_data[2];
        }


        void invert() {
            real det = determinant();

            real a = m_data[0];
            real b = m_data[1];
            real c = m_data[2];
            real d = m_data[3];

            m_data[0] =  d / det;
            m_data[1] = -b / det;
            m_data[2] = -c / det;
            m_data[3] =  a / det;
        }


        Matrix2 inverse() const {
            Matrix2 inv = *this;
            inv.invert();
            return inv;
        }


        Matrix2 operator+(const Matrix2& m) const {
            Matrix2 result;
            for (int i = 0; i < 4; i++) {
                result.m_data[i] = m_data[i] + m.m_data[i];
            }
            return result;
        }


        Matrix2 operator-(const Matrix2& m) const {
            Matrix2 result;
            for (int i = 0; i < 4; i++) {
                result.m_data[i] = m_data[i] - m.m_data[i];
            }
            return result;
        }


        Matrix2 operator*(const Matrix2& m) const {
            Matrix2 result;

            for (int r = 0; r < 2; r++) {
                for (int c = 0; c < 2; c++) {
                    result.m_data[r*2 + c] =
                        m_data[r*2 + 0] * m.m_data[0*2 + c] +
                        m_data[r*2 + 1] * m.m_data[1*2 + c];
                }
            }

            return result;
        }


        Vector2 operator*(const Vector2& v) const {
            return Vector2(
                m_data[0]*v.x() + m_data[1]*v.y(),
                m_data[2]*v.x() + m_data[3]*v.y()
            );
        }


        std::string toString() const {
            return "Matrix2("
                + std::to_string(m_data[0]) + " "
                + std::to_string(m_data[1]) + "\n"
                + std::to_string(m_data[2]) + " "
                + std::to_string(m_data[3]) + ")";
        }

    };

}

#endif