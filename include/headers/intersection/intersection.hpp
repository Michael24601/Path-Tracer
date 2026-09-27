
#ifndef PATH_TRACER_INTERSECTION_HPP
#define PATH_TRACER_INTERSECTION_HPP

#include "surfacePoint.hpp"

namespace pathtracer{

    // Represents a surface point we got by intersecting with a ray
    class Intersection: public SurfacePoint{

    private:

        // Distance along the ray that the intersection took place.
        real m_t;

    public:

        static Intersection NO_HIT;

        Intersection();

        Intersection(real t, const Vector3& position,
            const Vector3& geometryNormal, const Vector3& shadingNormal,
            const Vector3& tangent, const Vector2& uv, const Instance* instance);

        Intersection(real t, const SurfacePoint& sp);

        const real t() const;

        void setT(real t);

        // Returns false if not a hit
        explicit operator bool() const;
    };

}

#endif