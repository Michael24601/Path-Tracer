
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

    public:

        PathTracerGuided(int maxDepth);

        Vector3 color(const Ray& ray, const Scene& scene) override;

        std::string toString() const override{
            return "PT Guided ALPHA=" + std::to_string(m_alpha);
        }
    };

}

#endif