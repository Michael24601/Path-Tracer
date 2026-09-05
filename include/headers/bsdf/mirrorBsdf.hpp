
#ifndef PATH_TRACER_MIRROR_BSDF_HPP
#define PATH_TRACER_MIRROR_BSDF_HPP

#include "bsdf.hpp"
#include "../math/mathUtil.hpp"
#include "../core/random.hpp"
#include "../texture/texture.hpp"

namespace pathtracer{

    class MirrorBsdf: public Bsdf{

        real m_reflectance;

    public:

        MirrorBsdf(real reflectance) : m_reflectance(reflectance){}

        
        BsdfSample sample(const Vector3& wo, const Vector2& uv) 
            const override{

            // We only scatter light in the upper hemisphere.
            if(Bsdf::cosineTerm(wo) <= 0) {
                return BsdfSample::INVALID;
            }

            // Because this is a perfect mirror, we only need to
            // sample a single wi = wo reflected, which has a pdf of
            // infinity of being chosen, effectively meaning 
            // the integral is removed.

            Vector3 direction = reflect(wo);

            // The idea is that it does not contribute in any way,
            // so we'll place 1 as a placeholder.
            real pdf = 1.0;

            real cosine = cosineTerm(direction);

            Vector3 bsdf = m_reflectance;
            Vector3 weight = m_reflectance;

            return BsdfSample(bsdf, direction, cosine, pdf, weight);
        }


        BsdfSample evaluate(const Vector3& wo, 
            const Vector3& wi, const Vector2& uv) const override{

            // The probability that the given wi is the reflection of wo
            // is 0, so we return an invalid sample.
            return BsdfSample::INVALID;
        }
        

    };

}

#endif