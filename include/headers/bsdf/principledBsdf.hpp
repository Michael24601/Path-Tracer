#ifndef PATH_TRACER_PRINCIPLED_BSDF_HPP
#define PATH_TRACER_PRINCIPLED_BSDF_HPP

#include "bsdf.hpp"
#include "config.hpp"
#include "math/vector3.hpp"


namespace pathtracer{

    class Texture;

    // Not a bsdf subclass, just a helper
    class DiffuseLobe {
    
    private:

        // We don't need to store a texture here since this is not a bsdf.
        // The texture is sampled in the bsdf and the color for one pixel
        // is sent here.
        Vector3 m_color;

    public:

        DiffuseLobe(const Vector3& color);

        BsdfSample evaluate(const Vector3& wo, const Vector3& wi) const;

        BsdfSample sample(const Vector3& wo) const;

        const Vector3& color() const;
    };


    struct MetallicLobe {

    private:

        real m_alpha;
        
        // We don't need to store a texture here since this is not a bsdf.
        // The texture is sampled in the bsdf and the color for one pixel
        // is sent here.
        Vector3 m_color;

    public:

        MetallicLobe(const Vector3& color, real alpha);

        BsdfSample evaluate(const Vector3& wo, const Vector3& wi) const;

        BsdfSample sample(const Vector3& wo) const;

        const Vector3& color() const;

        real alpha() const;
    };


    class PrincipledBsdf : public Bsdf {

    private:

        Texture* m_baseColor;

        // Assumed greyscale
        Texture* m_roughness;
        Texture* m_metallic;
        Texture* m_specular;

        struct Combination {

            real diffuseSelectionProb;
            DiffuseLobe diffuse;
            MetallicLobe metallic;

            Combination(real diffuseSelectionProb,
                const DiffuseLobe& diffuse,
                const MetallicLobe& metallic);
        };

        Combination combine(const Vector2& uv, const Vector3& wo) const;

    public:

        PrincipledBsdf(Texture* baseColor, Texture* roughness,
            Texture* metallic, Texture* specular);

        BsdfSample evaluate(const Vector3& wo, const Vector3& wi,
            const Vector2& uv) const override;

        BsdfSample sample(const Vector3& wo, const Vector2& uv) const override;

        bool isSpecular() const override;
    };

}

#endif