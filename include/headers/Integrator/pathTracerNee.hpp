
#ifndef PATH_TRACER_PATH_TRACER_NEE_HPP
#define PATH_TRACER_PATH_TRACER_NEE_HPP

#include "integrator.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerNee: public Integrator{

    private:

        int m_maxDepth;

    public:

        PathTracerNee(int maxDepth);

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif