#ifndef PATH_TRACER_DIFFUSE_BSDF_HPP
#define PATH_TRACER_DIFFUSE_BSDF_HPP

#include "bsdf/bsdf.hpp"

namespace pathtracer{

    class Texture;

    class DiffuseBsdf: public Bsdf{

    private:

        // The color a this sample point could either come from a texture 
        // or from an albedo. It is set from outside.
        const Texture* m_albedo;

    public:

        DiffuseBsdf(const Texture* albedo);

        
        BsdfSample sample(const Vector3& wo, const Vector2& uv) 
            const override;


        BsdfSample evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const override;


        bool isSpecular() const override;
        
    };

}

#endif