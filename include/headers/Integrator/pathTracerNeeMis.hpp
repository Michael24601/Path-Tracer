
#ifndef PATH_TRACER_PATH_TRACER_NEE_MIS_HPP
#define PATH_TRACER_PATH_TRACER_NEE_MIS_HPP

#include "integrator.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerNeeMis: public Integrator{

    private:

        int m_maxDepth;

    public:

        // Uses both NEE and BSDF sampling, but instead of using one
        // or the other, blends them with MIS.
        PathTracerNeeMis(int maxDepth);

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif