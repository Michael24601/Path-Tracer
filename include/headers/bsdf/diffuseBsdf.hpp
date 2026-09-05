
#ifndef PATH_TRACER_DIFFUSE_BSDF_HPP
#define PATH_TRACER_DIFFUSE_BSDF_HPP

#include "bsdf.hpp"
#include "../math/mathUtil.hpp"
#include "../core/random.hpp"
#include "../texture/texture.hpp"

namespace pathtracer{

    class DiffuseBsdf: public Bsdf{

        // The color a this sample point could either come from a texture 
        // or from an albedo. It is set from outside.
        const Texture* m_albedo;

    public:

        DiffuseBsdf(const Texture* albedo) : m_albedo(albedo){}

        
        BsdfSample sample(const Vector3& wo, const Vector2& uv) 
            const override{

            // In a diffuse BSDF, we can just sample any
            // random direction in the hemisphere, and so long
            // as we sample uniformly, the pdf will be 1/2pi.
            Vector3 direction = 
                SquareToHemisphereCosine::transform(Random::next2D());
            direction.normalize();

            // The cosine term is the normal dot wi, and since we
            // are in local coordinates, the normal is the z axis.
            real cosine = Bsdf::cosineTerm(direction);

            // We only scatter light in the upper hemisphere
            // above the point, not below the point (inside the object).
            // So cosine should be positive.
            // Wi is sampled in teh upper hemisphere, so it's always
            // positive, but we need to check this for wo.
            if(Bsdf::cosineTerm(wo) <= 0) {
                return BsdfSample::INVALID;
            }

            // Cosine weighted
            real pdf = SquareToHemisphereCosine::pdf(direction);

            Vector3 albedo = m_albedo->sample(uv);
            Vector3 bsdf = albedo * INV_PI;

            // Since we know the cosines and PI cancel out, we can avoid
            // the division by the pdf.
            Vector3 weight = albedo; 

            return BsdfSample(bsdf, direction, cosine, pdf, weight);
        }


        BsdfSample evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const override{

            real pdf = SquareToHemisphereCosine::pdf(wi);
            Vector3 albedo = m_albedo->sample(uv);
            Vector3 bsdf = albedo * INV_PI;
            // The cosine term is the normal dot wi, and since we
            // are in local coordinates, the normal is the z axis.
            real cosine = Bsdf::cosineTerm(wi);

            // If the direction is the wrong way (diffuse
            // only scatters in upper hemisphere)
            // Otherwise, the shaded side of the objects would get
            // some light by intersecting the inside of the object,
            // on the lighted side.

            // In this case, both wi and wo are given to us,
            // so we make sure they are on the same side.
            if(Bsdf::cosineTerm(wo) <= 0 || cosine <= 0){
                return BsdfSample::INVALID;
            }

            Vector3 weight = albedo;

            return BsdfSample(bsdf, wi, cosine, pdf, weight);
        }
        

    };

}

#endif