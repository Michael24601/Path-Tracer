
#ifndef PATH_TRACER_MATRIX3_HPP
#define PATH_TRACER_MATRIX3_HPP

#include "../config.hpp"
#include "constants.hpp"

namespace pathtracer{

    class Matrix3{

    private:

        real m_data[9];
        
        
    public:

        static Matrix3 IDENTITY;    


        // Identity
        Matrix3(){ 
            m_data[1] = m_data[2] = m_data[3] = m_data[5] =
            m_data[6] = m_data[7] = 0.0; 
            m_data[0] = m_data[4] = m_data[8] = 1;
        }


        // These are assumed to be columns
        Matrix3(const Vector3& c0, const Vector3& c1, const Vector3& c2){
            m_data[0] = c0.x();
            m_data[1] = c1.x();
            m_data[2] = c2.x();
            m_data[3] = c0.y();
            m_data[4] = c1.y();
            m_data[5] = c2.y();
            m_data[6] = c0.z();
            m_data[7] = c1.z();
            m_data[8] = c2.z();
        }


        static Matrix3 rotationMatrix(real angle, int axis) {

            assert((axis >= 0 && axis < 3) && "Axis out of bounds");

            real c = std::cos(angle);
            real s = std::sin(angle);
            Matrix3 r;

            if (axis == 0){
                r = Matrix3(
                    Vector3(1, 0, 0),
                    Vector3(0, c, s),
                    Vector3(0,-s, c)
                );
            }
            else if (axis == 1){
                r = Matrix3(
                    Vector3( c, 0,-s),
                    Vector3( 0, 1, 0),
                    Vector3( s, 0, c)
                );
            }
            else {
                r = Matrix3(
                    Vector3( c, s, 0),
                    Vector3(-s, c, 0),
                    Vector3( 0, 0, 1)
                );
            }

            return r;
        }


        static Matrix3 scaleMatrix(real scale, int axis) {

            assert((axis >= 0 && axis < 3) && "Axis out of bounds");

            Matrix3 r;
            if (axis == 0){
                r.m_data[0] *= scale;
            }
            else if (axis == 1){
                r.m_data[4] *= scale;
            }
            else {
                r.m_data[8] *= scale;
            }

            return r;
        }


        // Column getter
        Vector3 column(int col) const{ 
            assert((col >= 0 && col < 3) && "Column out of bounds");
            return Vector3(m_data[col], m_data[col+3], m_data[col+6]);
        }


        void setColumn(const Vector3& v, int col){ 
            assert((col >= 0 && col < 3) && "Column out of bounds");
            m_data[col] = v.x();
            m_data[col+3] = v.y();
            m_data[col+6] = v.z();
        }


        const real& operator()(int row, int col) const {
            assert((row >= 0 && row < 3) && (col >= 0 && col < 3)
                && "Index out of bounds");
            return m_data[row * 3 + col];
        }


        real& operator()(int row, int col) {
            assert((row >= 0 && row < 3) && (col >= 0 && col < 3)
                && "Index out of bounds");
            return m_data[row * 3 + col];
        }


        void transpose(){
            Util::swap(m_data[1], m_data[3]);
            Util::swap(m_data[2], m_data[6]);
            Util::swap(m_data[5], m_data[7]);
        }


        Matrix3 transposed() const{
            Matrix3 t = *this;
            t.transpose();
            return t;
        }


        real determinant() const{
            Vector3 c0 = column(0);
            Vector3 c1 = column(1);
            Vector3 c2 = column(2);
            return c0.dot(c1.cross(c2));
        }


        void invert(){
            Vector3 c0 = column(0);
            Vector3 c1 = column(1);
            Vector3 c2 = column(2);

            Vector3 r0 = c1.cross(c2);
            Vector3 r1 = c2.cross(c0);
            Vector3 r2 = c0.cross(c1);

            // Better than calling the function, since that would
            // add another cross product.
            real det = c0.dot(r0);
            assert((std::abs(det) > EPSILON) && "Matrix is singular");
            real invDet = 1.0 / det;

            r0 = r0 * invDet;
            r1 = r1 * invDet;
            r2 = r2 * invDet;

            setColumn(r0, 0);
            setColumn(r1, 1);
            setColumn(r2, 2);
        }


        Matrix3 inverse() const{
            Matrix3 inv = *this;
            inv.invert();
            return inv; 
        }

        
        Matrix3 operator+(const Matrix3& m) const {
            Matrix3 result;
            for (int i = 0; i < 9; i++){ 
                result.m_data[i] = m_data[i] + m.m_data[i];
            }
            return result;
        }


        Matrix3 operator-(const Matrix3& m) const {
            Matrix3 result;
            for (int i = 0; i < 9; i++){ 
                result.m_data[i] = m_data[i] - m.m_data[i];
            }
            return result;
        }


        Matrix3 operator*(const Matrix3& m) const {
            Matrix3 result;
            // More efficients than doing a tranpose and 3 dot
            // products.
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    result.m_data[r*3 + c] = 
                        m_data[r*3 + 0] * m.m_data[0*3 + c] +
                        m_data[r*3 + 1] * m.m_data[1*3 + c] +
                        m_data[r*3 + 2] * m.m_data[2*3 + c];
                }
            }
            return result;
        }


        Vector3 operator*(const Vector3& v) const {
            return Vector3(
                m_data[0]*v.x() + m_data[1]*v.y() + m_data[2]*v.z(),
                m_data[3]*v.x() + m_data[4]*v.y() + m_data[5]*v.z(),
                m_data[6]*v.x() + m_data[7]*v.y() + m_data[8]*v.z()
            );
        }


        void addRotation(real angle, int axis){
            Matrix3 r = rotationMatrix(angle, axis);
            *this = r * *this;
        }


        void scale(real scale, int axis){
            Matrix3 s = scaleMatrix(scale, axis);
            *this = s * *this;
        }

    };

}

#endif