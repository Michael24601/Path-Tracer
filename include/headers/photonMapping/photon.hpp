
#ifndef PATH_TRACER_PHOTON_HPP
#define PATH_TRACER_PHOTON_HPP

#include <vector>
#include "intersection/surfacePoint.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    struct Photon{

        // The radiance from the light at the photon
        Vector3 weight;
        // Direction towards light
        Vector3 wi;
        // The surface point of the photon
        SurfacePoint point;
    };

}

#endif