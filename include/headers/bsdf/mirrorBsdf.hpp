#ifndef PATH_TRACER_MIRROR_BSDF_HPP
#define PATH_TRACER_MIRROR_BSDF_HPP

#include "specular.hpp"
#include "bsdf.hpp"

namespace pathtracer{

    class MirrorBsdf: public Bsdf, public Specular{

        real m_reflectance;

    public:

        MirrorBsdf(real reflectance);

        BsdfSample sample(const Vector3& wo, const Vector2& uv) 
            const override;

        BsdfSample evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const override;

        bool isSpecular() const override;        

        // This evaluates the weight for reflecting (no wi is sent since
        // only one works).
        Vector3 evaluateReflection(const Vector3& wo,
            const Vector2& uv) const override;

        // This always returns 0.0 since the pure mirror does not refract.
        Vector3 evaluateRefraction(const Vector3& wo, 
            const Vector2& uv) const override;

        // Mirrors can't refract so ior can be 1.0 (dummy value)
        real eta(const Vector3& wo, const Vector2& uv) const override;
    };

}

#endif