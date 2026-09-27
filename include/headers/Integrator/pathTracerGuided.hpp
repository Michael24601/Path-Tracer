
#ifndef PATH_TRACER_PATH_TRACER_GUIDED_HPP
#define PATH_TRACER_PATH_TRACER_GUIDED_HPP

#include <atomic>
#include "integrator.hpp"
#include "guidable.hpp"
#include "config.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerGuided: public Integrator, public Guidable {

    private:

        int m_maxDepth;

        // Weight used when combining bsdf sampling with guiding
        // (defensive sampling).
        real m_alpha;

    public:

        static std::atomic<long long> n1, n2;

        PathTracerGuided(int maxDepth, real alpha);

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif