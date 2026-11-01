
#include "sms/smsSample.hpp"
#include "intersection/surfaceDifferentials.hpp"

namespace pathtracer{

    SmsSample::SmsSample(const SurfacePoint& finalPoint, 
        const SurfaceDifferentials& d,
        const Vector3& halfVector, real eta, 
        bool isReflection, bool isConverged) :
        m_finalPoint{finalPoint},
        m_differentials{d},
        m_halfVector{halfVector},
        m_eta{eta},
        m_isReflection{isReflection},
        m_isConverged{isConverged} {
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

    const SurfaceDifferentials& SmsSample::differentials() const{
        return m_differentials;
    }

}