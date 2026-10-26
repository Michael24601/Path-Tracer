
#ifndef PATH_TRACER_DIELECTRIC_BSDF_HPP
#define PATH_TRACER_DIELECTRIC_BSDF_HPP

#include "specularManifoldBsdf.hpp"
#include "bsdf.hpp"

namespace pathtracer{

    class Texture;
    class Vector2;
    class Vector3;
    class BsdfSample;

    class DielectricBsdf : public Bsdf, public SpecularManifoldBsdf{

    private:

        // The color a this sample point could either come from a texture
        // or from an albedo. It is set from outside.
        const Texture* m_reflectance;
        const Texture* m_transmittance;
        const Texture* m_ior;


    public:


        DielectricBsdf(const Texture *ior, const Texture *reflectance,
            const Texture *transmittance);


        BsdfSample sample(const Vector3 &wo, const Vector2 &uv,
            bool adjoint = false) const override;


        BsdfSample evaluate(const Vector3 &wo, const Vector3 &wi, 
            const Vector2 &uv,  bool adjoint = false) const override;


        bool isSpecular() const override{
            return true;
        }


        // This evaluates the bsdf for reflecting (no wi is sent since
        // only one works). Same logic as the sample function.
        // Assumes reflection was actually chosen.
        Vector3 evaluateReflection(const Vector3& wo, const Vector3& wi,
            const Vector2& uv) const override;


        Vector3 evaluateRefraction(const Vector3& wo, const Vector3& wi,
            const Vector2& uv) const override;


        real eta(const Vector3& wo, const Vector2& uv) const override;


        // It is not rough so should return false
        bool isRough() const override{
            return false;
        }

        virtual ManifoldNormal sampleManifoldNormal(const Vector2& uv) const 
            override{

            // In local space, the shading normal is always this, since no
            // microfacets. Pdf is 1.0 since it's always the same.
            return ManifoldNormal{ Vector3(0.0, 0.0, 1.0), 1.0 };
        }


        bool supportsReflection(const Vector2& uv) const override;

        
        bool supportsRefraction(const Vector2& uv) const override;

    };

}

#endif