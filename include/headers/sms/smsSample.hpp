
#ifndef PATH_TRACER_SMS_SAMPLE_HPP
#define PATH_TRACER_SMS_SAMPLE_HPP

#include "intersection/surfacePoint.hpp"

namespace pathtracer{

    // Sample that contains the seed used to initialize SMS, and the
    // converged final point (if found).
    class SmsSample{

    private:

        SurfacePoint m_finalPoint;

        SurfaceDifferentials m_differentials;

        // We also keep track of eta and half vector.
        // The halfvector is not normalized.
        Vector3 m_halfVector;
        real m_eta;

        bool m_isConverged;
        bool m_isReflection;

    public:

        SmsSample(const SurfacePoint& finalPoint, const SurfaceDifferentials& d, 
            const Vector3& halfVector, real eta, bool isReflection, 
            bool isConverged);


        const SurfacePoint& finalPoint() const;

        const Vector3& halfVector() const;

        real eta() const;

        bool isConverged() const;

        bool isReflection() const;

        const SurfaceDifferentials& differentials() const;
    };

}

#endif