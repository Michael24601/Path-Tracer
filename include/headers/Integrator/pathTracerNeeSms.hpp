
#ifndef PATH_TRACER_PATH_TRACER_NEE_SMS_HPP
#define PATH_TRACER_PATH_TRACER_NEE_SMS_HPP

#include "integrator.hpp"

namespace pathtracer{

    class SpecularManifoldSampling;

    // A pure path tracer
    class PathTracerNeeSms: public Integrator{

    private:

        int m_maxDepth;

    public:

        SpecularManifoldSampling* sms;

        // The boolean determines which constraint is used for SMS
        PathTracerNeeSms(int maxDepth, bool useHalfVector, const Scene* scene);

        ~PathTracerNeeSms();

        Vector3 color(const Ray& ray, const Scene& scene) override;
    };

}

#endif