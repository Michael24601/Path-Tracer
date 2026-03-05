
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
                SquareToHemisphereUniform::transform(Random::next2D());
            direction.normalize();
            real pdf = SquareToHemisphereUniform::pdf(direction);
            Vector3 bsdf = m_albedo->sample(uv) * INV_PI;
            // The cosine term is the normal dot wi, and since we
            // are in local coordinates, the normal is the z axis.
            real cosine = Bsdf::cosineTerm(direction);

            return BsdfSample(bsdf, direction, cosine, pdf);
        }


        BsdfEvaluation evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const override{

            Vector3 bsdf = m_albedo->sample(uv) * INV_PI;
            return BsdfEvaluation(bsdf, false);
        }
        

    };

}

#endif