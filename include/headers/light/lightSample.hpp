
#ifndef PATH_TRACER_LIGHT_SAMPLE_HPP
#define PATH_TRACER_LIGHT_SAMPLE_HPP

#include "../math/vector3.hpp"
#include "../logger.hpp"

namespace pathtracer{

    class Light;
    
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

        // The cosine at the hit, only really meaningful at an area light
        real m_cosine;
        
        // Distance to light
        real m_distance;

        // Optional parameter when light intersected is a mesh
        int m_triangleIndex;

        const Light* m_caster;


    public:

        static LightSample INVALID;

        LightSample(const Vector3& wi, const Vector3& radiance,
            const Vector3& position, real pdf, real distance, const Light* caster,
            real cosine = 0.0, int triangleIndex = -1): 
            m_wi(wi), m_radiance(radiance), m_position(position), 
            m_pdf(pdf), m_distance{distance}, m_caster{caster}, 
            m_cosine{cosine}, m_triangleIndex{triangleIndex}{}


        real pdf() const { return m_pdf; }


        const Vector3& wi() const { return m_wi; }


        const real cosine() const { return m_cosine; }


        const Vector3& radiance() const { return m_radiance; }


        const Vector3& position() const { return m_position; }


        real distance() const { return m_distance; }
        

        bool isValid() const { return m_distance > 0.0; }


        const Light* caster() const { return m_caster; }


        int triangleIndex() const { return m_triangleIndex; }

    };

}

#endif