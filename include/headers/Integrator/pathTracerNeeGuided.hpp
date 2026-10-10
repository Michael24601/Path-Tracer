
#ifndef PATH_TRACER_PATH_TRACER_NEE_GUIDED_HPP
#define PATH_TRACER_PATH_TRACER_NEE_GUIDED_HPP

#include "integrator.hpp"
#include "guidable.hpp"
#include "config.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerNeeGuided: public Integrator, public Guidable{

    private:

        int m_maxDepth;

    public:

        PathTracerNeeGuided(int maxDepth);

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif