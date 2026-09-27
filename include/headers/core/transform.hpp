#ifndef PATH_TRACER_TRANSFORM_HPP
#define PATH_TRACER_TRANSFORM_HPP

#include <string>
#include "math/matrix3.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    class Ray;
    class SurfacePoint;
    class SurfaceDifferentials;

    class Transform{

    private:

        // Scaling and rotation
        Matrix3 m_transform;
        // Translation
        Vector3 m_translation;

    public:

        static Transform IDENTITY;

        // Identity matrix and no translation
        Transform();

        Transform(const Vector3& angles, const Vector3& scale,
            const Vector3& translation);

        Transform(const Matrix3& transform, const Vector3& translation);

        const Matrix3& getMatrix() const;

        Transform inverse() const;

        // Returns the inverse tranpose without the translation
        Transform inverseTranspose() const;

        // Transforms a point
        Vector3 transform(const Vector3& point) const;

        const Matrix3& transform() const;

        const Vector3& translation() const;

        // Transforms a point
        Vector3 inverseTransform(const Vector3& point) const;

        // Transforms a direction (no translation, must remain normal)
        Vector3 transformDirection(const Vector3& direction) const;

        Vector3 transformDirectionKeepScale(const Vector3& direction) const;

        // Transforms a direction
        Vector3 inverseTransformDirection(const Vector3& direction) const;

        // Transforms a normal (must remain orthogonal),
        // so we use the inverse transposed
        Vector3 transformNormal(const Vector3& normal) const;

        // Transforms a normal but keeps the scale
        Vector3 transformNormalKeepScale(const Vector3& normal) const;

        // Inverse transforms a normal (from world to local)
        Vector3 inverseTransformNormal(const Vector3& normal) const;

        // Transforms a ray
        Ray transform(const Ray& ray) const;

        // Transforms a ray but does not normalize the direction
        Ray transformKeepScale(const Ray& ray) const;

        Ray inverseTransform(const Ray& ray) const;

        // Transforms surface points (like intersections)
        SurfacePoint transformSurfacePoint(const SurfacePoint& it) const;

        // Transforms surface differentials
        SurfaceDifferentials transformDifferentials(
            const SurfaceDifferentials& d, const Vector3& localNormal,
            const Vector3& worldNormal) const;

        // Returns the determinant of the transform matrix
        real determinant() const;

        std::string toString() const;
    };

}

#endif