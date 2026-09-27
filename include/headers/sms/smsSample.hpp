
#ifndef PATH_TRACER_SMS_SAMPLE_HPP
#define PATH_TRACER_SMS_SAMPLE_HPP

#include "intersection/surfacePoint.hpp"

namespace pathtracer{

    // Sample that contains the seed used to initialize SMS, and the
    // converged final point (if found).
    class SmsSample{

    private:

        SurfacePoint m_seedPoint;
        SurfacePoint m_finalPoint;

        // We also keep track of eta and half vector.
        // The halfvector is not normalized.
        Vector3 m_halfVector;
        real m_eta;

        bool m_isConverged;
        bool m_isReflection;

    public:

        SmsSample(const SurfacePoint& seedPoint, 
            const SurfacePoint& finalPoint,  const Vector3& halfVector, 
            real eta, bool isReflection, bool isConverged);

        const SurfacePoint& seedPoint() const;

        const SurfacePoint& finalPoint() const;

        const Vector3& halfVector() const;

        real eta() const;

        bool isConverged() const;

        bool isReflection() const;
    };

}

#endif