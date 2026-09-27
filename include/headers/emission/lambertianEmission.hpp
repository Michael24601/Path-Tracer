
#ifndef PATH_TRACER_LAMBERTIAN_EMISSION_HPP
#define PATH_TRACER_LAMBERTIAN_EMISSION_HPP

#include "emission.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    // This is an emission that radiates uniformally in a hemisphere
    // above the intersected point.
    class LambertianEmission: public Emission{

    private:

        Vector3 m_emissionColor;

    public:

        LambertianEmission(const Vector3& emissionColor);

        Vector3 evaluate(const Vector3& wo, const Vector2& uv)
            const override;
    };

}

#endif