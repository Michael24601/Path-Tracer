
#include "bsdf/diffuseBsdf.hpp"
#include "math/mathUtil.hpp"
#include "core/random.hpp"
#include "texture/texture.hpp"
#include "bsdf/bsdfSample.hpp"
#include "math/constants.hpp"
#include "math/vector2.hpp"


namespace pathtracer{


    DiffuseBsdf::DiffuseBsdf(const Texture* albedo) : m_albedo(albedo){}


    BsdfSample DiffuseBsdf::sample(const Vector3& wo, const Vector2& uv) const {

        // In a diffuse BSDF, we can just sample any
        // random direction in the hemisphere, and so long
        // as we sample uniformly, the pdf will be 1/2pi.
        Vector3 direction = 
            SquareToHemisphereCosine::transform(Random::next2D());
        direction.normalize();

        // We only scatter light in the upper hemisphere
        // above the point, not below the point (inside the object).
        // So cosine should be positive.
        // Wi is sampled in teh upper hemisphere, so it's always
        // positive, but we need to check this for wo.
        if(ShadingSpace::cosineTheta(wo) <= 0) {
            direction = -direction;
        }

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        real cosine = ShadingSpace::absCosineTheta(direction);

        // Cosine weighted
        real pdf = SquareToHemisphereCosine::pdf(direction);

        Vector3 albedo = m_albedo->sample(uv);
        Vector3 bsdf = albedo * INV_PI;

        // Since we know the cosines and PI cancel out, we can avoid
        // the division by the pdf.
        Vector3 weight = albedo;

        return BsdfSample(bsdf, direction, cosine, pdf, weight);
    }


    BsdfSample DiffuseBsdf::evaluate(const Vector3& wo, const Vector3& wi, 
        const Vector2& uv) const{

        real pdf = SquareToHemisphereCosine::pdf(wi);

        Vector3 albedo = m_albedo->sample(uv);
        Vector3 bsdf = albedo * INV_PI;
        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        real cosine = ShadingSpace::cosineTheta(wi);

        // If the direction is the wrong way (diffuse
        // only scatters in upper hemisphere)
        // Otherwise, the shaded side of the objects would get
        // some light by intersecting the inside of the object,
        // on the lighted side.

        Vector3 weight = albedo;

        // In this case, both wi and wo are given to us,
        // so we make sure they are on the same side.
        if(ShadingSpace::cosineTheta(wo) * cosine < 0){
            return BsdfSample::INVALID;            
        }

        return BsdfSample(bsdf, wi, cosine, pdf, weight);
    }


    bool DiffuseBsdf::isSpecular() const {
        return false;
    }

}