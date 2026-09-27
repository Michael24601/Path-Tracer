
#ifndef PATH_TRACER_MATRIX2_HPP
#define PATH_TRACER_MATRIX2_HPP

#include "config.hpp"
#include <string>

namespace pathtracer{

    // Forward declaration
    class Vector2;
    

    class Matrix2{

    private:

        real m_data[4];

    public:

        static Matrix2 IDENTITY;

        // Identity
        Matrix2();

        // These are assumed to be columns
        Matrix2(real c0, real c1, real c2, real c3);

        // These are assumed to be columns
        Matrix2(const Vector2& c0, const Vector2& c1);

        static Matrix2 rotationMatrix(real angle);

        // Column getter
        Vector2 column(int col) const;

        void setColumn(const Vector2& v, int col);

        const real& operator()(int row, int col) const;

        real& operator()(int row, int col);

        void transpose();

        Matrix2 transposed() const;

        real determinant() const;

        void invert();

        Matrix2 inverse() const;

        Matrix2 operator+(const Matrix2& m) const;

        Matrix2 operator-(const Matrix2& m) const;

        Matrix2 operator*(const Matrix2& m) const;

        Vector2 operator*(const Vector2& v) const;

        std::string toString() const;

    };

}

#endif