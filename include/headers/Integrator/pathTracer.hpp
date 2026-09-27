
#ifndef PATH_TRACER_PATH_TRACER_HPP
#define PATH_TRACER_PATH_TRACER_HPP

#include "integrator.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracer: public Integrator{

    private:

        int m_maxDepth;

    public:

        PathTracer(int maxDepth);

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif