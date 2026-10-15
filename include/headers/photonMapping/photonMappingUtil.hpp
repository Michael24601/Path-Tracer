
#ifndef PATH_TRACER_PHOTON_MAPPING_UTIL_HPP
#define PATH_TRACER_PHOTON_MAPPING_UTIL_HPP

namespace pathtracer{

    class PhotonMap;
    class Scene;

    namespace PhotonMapping{

        // The photon count is the number of photons.
        // The maxDepth is the ray tracing limit.
        // The threshold is for the photon map kdTree subdivision rule.
        PhotonMap* tracePhotons(int photonCount, int maxDepth, int threshold,
            const Scene& scene);

    }
    
}

#endif