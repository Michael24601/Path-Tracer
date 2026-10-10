
#ifndef PATH_TRACER_PATH_TRACER_NEE_MIS_GUIDED_HPP
#define PATH_TRACER_PATH_TRACER_NEE_MIS_GUIDED_HPP

#include "integrator.hpp"
#include "guidable.hpp"
#include "config.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerNeeMisGuided: public Integrator, public Guidable {

    private:

        int m_maxDepth;

    public:

        // Uses both NEE and BSDF sampling, but instead of using one
        // or the other, blends them with MIS.
        PathTracerNeeMisGuided(int maxDepth);

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif