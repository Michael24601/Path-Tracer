
#ifndef PATH_TRACER_LIGHT_SAMPLE_HPP
#define PATH_TRACER_LIGHT_SAMPLE_HPP

#include "../math/vector3.hpp"

namespace pathtracer{

    
    class LightSample{

    private:

        // The pdf of having chosen the specific position 
        // on the light (given that the light was chosen),
        // in solid angles.
        real m_pdf;

        // The direction from which the light was sampled.
        Vector3 m_wi;

        // The radiance
        Vector3 m_radiance;

        // The sampled point
        Vector3 m_position;
        
        // Distance to light
        real m_distance;

    public:

        static LightSample INVALID;

        LightSample(const Vector3& wi, const Vector3& radiance,
            const Vector3& position, real pdf, real distance): 
            m_wi(wi), m_radiance(radiance), m_position(position), 
            m_pdf(pdf), m_distance{distance}{}


        real pdf() const { return m_pdf; }


        const Vector3& wi() const { return m_wi; }


        const Vector3& radiance() const { return m_radiance; }


        const Vector3& position() const { return m_position; }


        real distance() const { return m_distance; }
        

    };

}

#endif