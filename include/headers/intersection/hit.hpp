
#ifndef PATH_TRACER_HIT_HPP
#define PATH_TRACER_HIT_HPP

#include "math/vector2.hpp"
#include "core/instance.hpp"

namespace pathtracer{

    // This is a Hit object, which is used to test for intersections
    // without constructing a full SurfacePoint.
    struct Hit{
        // Distance along ray in local coordinates (at start)
        real t;
        // The hit instance
        Instance* instance;

        // Barycentric coordinates (for triangle only)
        Vector2 barycentric;
        // Triangle index (for mesh only)
        int triangleIndex;
    };

}

#endif