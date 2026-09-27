
#include "sms/smsSample.hpp"

namespace pathtracer{

    SmsSample::SmsSample(const SurfacePoint& seedPoint, 
        const SurfacePoint& finalPoint, const Vector3& halfVector, real eta, 
        bool isReflection, bool isConverged) :
        m_seedPoint{seedPoint},
        m_finalPoint{finalPoint},
        m_halfVector{halfVector},
        m_eta{eta},
        m_isReflection{isReflection},
        m_isConverged{isConverged} {
    }

    const SurfacePoint& SmsSample::seedPoint() const {
        return m_seedPoint;
    }

    const SurfacePoint& SmsSample::finalPoint() const {
        return m_finalPoint;
    }

    const Vector3& SmsSample::halfVector() const {
        return m_halfVector;
    }

    real SmsSample::eta() const {
        return m_eta;
    }

    bool SmsSample::isConverged() const {
        return m_isConverged;
    }

    bool SmsSample::isReflection() const {
        return m_isReflection;
    }

}