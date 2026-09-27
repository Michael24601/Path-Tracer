#ifndef PATH_TRACER_AXIS_ALIGNED_BOX_HPP
#define PATH_TRACER_AXIS_ALIGNED_BOX_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    class Ray;

    class AxisAlignedBox{

    private:

        Vector3 m_minCorner;
        Vector3 m_maxCorner;

    public:

        AxisAlignedBox();

        AxisAlignedBox(const Vector3& minCorner, const Vector3& maxCorner);

        // Increases the AABB size to fit another AABB
        void extend(const AxisAlignedBox& bounds);

        // Increases AABB size to fit a point
        void extend(const Vector3& point);

        real surfaceArea() const;

        const Vector3& minCorner() const;

        const Vector3& maxCorner() const;

        real intersect(const Ray& ray) const;

    };

}

#endif