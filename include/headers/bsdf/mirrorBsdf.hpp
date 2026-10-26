
#ifndef PATH_TRACER_MIRROR_BSDF_HPP
#define PATH_TRACER_MIRROR_BSDF_HPP

#include "specularManifoldBsdf.hpp"
#include "bsdf.hpp"

namespace pathtracer{

    class Texture;

    class MirrorBsdf: public Bsdf, public SpecularManifoldBsdf{

        Texture* m_reflectance;

    public:

        MirrorBsdf(Texture* reflectance);

        BsdfSample sample(const Vector3 &wo, const Vector2 &uv,
            bool adjoint = false) const override;


        BsdfSample evaluate(const Vector3 &wo, const Vector3 &wi, 
            const Vector2 &uv,  bool adjoint = false) const override;


        bool isSpecular() const override{
            return true;
        }   


        // This evaluates the weight for reflecting (no wi is sent since
        // only one works).
        Vector3 evaluateReflection(const Vector3& wo,
            const Vector3& wi, const Vector2& uv) const override;

            
        // This always returns 0.0 since the pure mirror does not refract.
        Vector3 evaluateRefraction(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const override;


        // Mirrors can't refract so ior can be 1.0 (dummy value)
        real eta(const Vector3& wo, const Vector2& uv) const override{
            return 1.0;
        }


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


        bool supportsReflection(const Vector2& uv) const override{
            return true;
        }

        
        bool supportsRefraction(const Vector2& uv) const override{
            return false;
        }
    };

}

#endif