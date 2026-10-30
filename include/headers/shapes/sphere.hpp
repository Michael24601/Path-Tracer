
#ifndef PATH_TRACER_SPHERE_HPP
#define PATH_TRACER_SPHERE_HPP

#include "shape.hpp"

namespace pathtracer{

    class SurfacePoint;
    class Hit;

    // The sphere is in local coordinates, so we assume the
    // center is at the origin, and the radius is 1.
    class Sphere : public Shape{

    private:

        // Generates the point with its shading frame and texture coordinates
        SurfacePoint generateSurfacePoint(const Vector3& point) const;

    public:

        Sphere();

        real getSurfaceArea() const override;

        AxisAlignedBox getBoundingBox() const override;

        Vector3 getCentroid() const override;

        // Intersects the shape with a ray
        bool intersect(const Ray&, Hit& oldHit) const override;

        AreaSample sampleSurfaceArea() const override;

        AreaSample evaluateAreaSample(const SurfaceSample& point) const override;

        Vector3 getPosition(const Vector2& uv, int triangleIndex) const override;

        Vector2 getUv(const Hit& hit, const Ray& ray) const override;

        SurfaceDifferentials computeDifferentials(const Vector3& position, 
            const Vector2& uv) const;


        SurfacePoint getSurfacePoint(const Vector3& point,
            const Hit& hit) const override;
    };

}

#endif