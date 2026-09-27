
#ifndef PATH_TRACER_MESH_HPP
#define PATH_TRACER_MESH_HPP

#include "triangle.hpp"
#include <vector>
#include "bvh/axisAlignedBox.hpp"

namespace pathtracer{

    class BlAccelerationStructure;

    class Mesh : public Shape {

    private:

        // A mesh is just a collection of triangles
        std::vector<Triangle> triangles;

        // Surface area and bounding box stored since expensive to compute
        // on the fly each time it's needed.
        real m_surfaceArea;
        AxisAlignedBox m_box;

        BlAccelerationStructure* m_bvh;

    public:

        Mesh(const std::vector<Triangle>& triangles);

        ~Mesh();

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