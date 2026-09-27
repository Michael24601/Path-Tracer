#ifndef PATH_TRACER_MATRIX3_HPP
#define PATH_TRACER_MATRIX3_HPP

#include <string>
#include "config.hpp"

namespace pathtracer{

    class Vector3;

    class Matrix3{

    private:

        real m_data[9];

    public:

        static Matrix3 IDENTITY;

        // Identity
        Matrix3();

        // These are assumed to be columns
        Matrix3(const Vector3& c0, const Vector3& c1, const Vector3& c2);

        static Matrix3 rotationMatrix(real angle, int axis);

        static Matrix3 scaleMatrix(real scale, int axis);

        // Column getter
        Vector3 column(int col) const;

        void setColumn(const Vector3& v, int col);

        const real& operator()(int row, int col) const;

        real& operator()(int row, int col);

        void transpose();

        Matrix3 transposed() const;

        real determinant() const;

        void invert();

        Matrix3 inverse() const;

        Matrix3 operator+(const Matrix3& m) const;

        Matrix3 operator-(const Matrix3& m) const;

        Matrix3 operator*(const Matrix3& m) const;

        Vector3 operator*(const Vector3& v) const;

        void addRotation(real angle, int axis);

        void scale(real scale, int axis);

        std::string toString() const;

    };

}

#endif