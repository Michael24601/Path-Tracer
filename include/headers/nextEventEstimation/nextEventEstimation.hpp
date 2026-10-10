
#ifndef PATH_TRACER_NEXT_EVENT_ESTIMATION_HPP
#define PATH_TRACER_NEXT_EVENT_ESTIMATION_HPP

#include "config.hpp"
#include "light/lightSample.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    class Scene;
    class SurfacePoint;

    struct NextEventSample{
        LightSample sample;
        bool visibility;
        // Contribution already includes the pdf
        Vector3 contribution;
        // But we also return pdf in case it's needed
        real pdf;
    };

    // Single scatter event SMS
    namespace NextEventEstimation{

        // Samples a light, and returns the light sample, the contribution
        // (weighted by the solid angle pdf), and the visibility.
        // The surface point is the origin point sampling the light.
        NextEventSample sample(const Scene& scene, const SurfacePoint& origin);


        // In the same way we sample and evaluate bsdf, we can evaluate
        // an NEE sample given the point and the light.
        NextEventSample evaluate(const Scene& scene, const Vector3& origin,
            const SurfacePoint& emissive);

    }

}

#endif