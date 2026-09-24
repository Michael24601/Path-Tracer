

#ifndef PATH_TRACER_SMS_SAMPLE_HPP
#define PATH_TRACER_SMS_SAMPLE_HPP

#include "../core/instance.hpp"

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


        SmsSample(const SurfacePoint& seedPoint, const SurfacePoint& finalPoint,
            const Vector3& halfVector, real eta,
            bool isReflection, bool isConverged) : m_seedPoint{seedPoint},
            m_finalPoint{finalPoint}, m_halfVector{halfVector}, m_eta{eta},
            m_isReflection{isReflection}, m_isConverged{isConverged}{}


        const SurfacePoint& seedPoint() const{
            return m_seedPoint;
        }

        const SurfacePoint& finalPoint() const{
            return m_finalPoint;
        }

        const Vector3& halfVector() const {
            return m_halfVector;
        }

        real eta() const{
            return m_eta;
        }

        bool isConverged() const{
            return m_isConverged;
        }

        bool isReflection() const{
            return m_isReflection;
        }

    };

}


#endif