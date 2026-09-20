

#ifndef PATH_TRACER_SMS_SAMPLE_HPP
#define PATH_TRACER_SMS_SAMPLE_HPP

#include "../core/instance.hpp"

namespace pathtracer{

    // Sample that contains the seed used to initialize SMS, and the
    // converged final point (if found).
    class SmsSample{

    private:


        Vector3 m_seedPoint;
        Vector3 m_finalPoint;
        bool m_isConverged;
        

    public:


        SmsSample(const Vector3& seedPoint, const Vector3& finalPoint, 
            bool isConverged) : m_seedPoint{seedPoint},
            m_finalPoint{finalPoint}, m_isConverged{isConverged}{}


        const Vector3& seedPoint() const{
            return m_seedPoint;
        }

        const Vector3& finalPoint() const{
            return m_finalPoint;
        }

        bool isConverged() const{
            return m_isConverged;
        }

    };

}


#endif