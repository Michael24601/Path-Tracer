#ifndef PATH_TRACER_TRIANGLE_HPP
#define PATH_TRACER_TRIANGLE_HPP

#include "shape.hpp"
#include "math/vector2.hpp"

namespace pathtracer{

    class SurfacePoint;

    class Triangle : public Shape{

    private:

        // The triangle is stored using 3 vertices in global
        // coordinates, along with 3 normals that may or may not be
        // given.
        Vector3 v0, v1, v2;
        Vector3 n0, n1, n2;
        // UV coordinates of each triangle vertex
        Vector2 uv0, uv1, uv2;

        // True if we have shading normals and texture coordinates
        bool m_shadingNormals;
        bool m_uvCoordinates;


        // Given a ray and a distance t along it, sets the intersection
        // object. The uv are the barycentric coordinates of the
        // intersected point.
        SurfacePoint generateSurfacePoint(const Ray& ray, real t,
            const Vector2& barycentric) const;

    public:

        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2);


        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector3& n0, const Vector3& n1,const Vector3& n2);


        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector2& uv0, const Vector2& uv1, const Vector2& uv2);

            
        Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
            const Vector3& n0, const Vector3& n1, const Vector3& n2,
            const Vector2& uv0, const Vector2& uv1, const Vector2& uv2);


        real getSurfaceArea() const override;


        AxisAlignedBox getBoundingBox() const override;


        Vector3 getCentroid() const override;


        // Intersects the shape with a ray
        Intersection intersect(const Ray& ray, real oldT) const override;


        AreaSample sampleSurfaceArea() const override;


        AreaSample evaluateAreaSample(const SurfaceSample& point) const override;


        SurfaceDifferentials computeDifferentials(const Vector3& position,
            const Vector3& shadingNormal, const Vector2& uv,
            int triangleIndex) const override;


        Vector3 getPosition(const Vector2& uv, int triangleIndex) const override;
    };

}

#endif