#ifndef PATH_TRACER_AOV_INTEGRATOR_HPP
#define PATH_TRACER_AOV_INTEGRATOR_HPP

#include "integrator.hpp"

namespace pathtracer{

    class AovIntegrator: public Integrator{

    public:

        enum class RenderVariable{
            ALBEDO,
            NORMAL,
            DIRECTION,
            DPDU,
            DPDV,
            DNDU,
            DNDV,
            DSDU,
            DSDV,
            DTDU,
            DTDV,
            S
        };

    private:

        // Choice of variable to output, if we happen to intersect
        // an object.
        RenderVariable m_variable;

    public:

        AovIntegrator(RenderVariable variable);

        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif