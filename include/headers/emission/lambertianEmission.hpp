
#ifndef PATH_TRACER_LAMBERTIAN_EMISSION_HPP
#define PATH_TRACER_LAMBERTIAN_EMISSION_HPP

#include "emission.hpp"

namespace pathtracer{

    // This is an emission that radiates uniformally in a hemisphere
    // above the intersected point.
    class LambertianEmission: public Emission{\

    private:
    
        Vector3 m_emissionColor;

    public:

        Vector3 evaluate(const Vector3& wo, const Vector2& uv) 
            const override{

            // The emission in wo is weighted by the cosine of the
            // angle the normal makes with the outgoing ray of light.
            real cosine = Emission::cosineTerm(wo);
            return m_emissionColor * cosine;
        };

    };

}

#endif