
#include "light/lightSample.hpp"

namespace pathtracer{


    LightSample LightSample::INVALID = LightSample(Vector3(0.0), 
        Vector3(0.0), Vector3(0.0), 0.0, 0.0, nullptr);


    LightSample::LightSample(const Vector3& wi, const Vector3& radiance,
        const Vector3& position, real pdf, real distance, 
        const Light* caster, real cosine, int triangleIndex) :
        m_wi(wi), m_radiance(radiance), m_position(position),
        m_pdf(pdf), m_distance{distance}, m_caster{caster},
        m_cosine{cosine}, m_triangleIndex{triangleIndex} {}


    real LightSample::pdf() const {
        return m_pdf;
    }


    const Vector3& LightSample::wi() const {
        return m_wi;
    }


    const real LightSample::cosine() const {
        return m_cosine;
    }


    const Vector3& LightSample::radiance() const {
        return m_radiance;
    }


    const Vector3& LightSample::position() const {
        return m_position;
    }


    real LightSample::distance() const {
        return m_distance;
    }


    bool LightSample::isValid() const {
        return m_distance > 0.0;
    }


    const Light* LightSample::caster() const {
        return m_caster;
    }

    
    int LightSample::triangleIndex() const {
        return m_triangleIndex;
    }

}