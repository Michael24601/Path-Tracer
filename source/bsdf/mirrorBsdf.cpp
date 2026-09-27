#include "bsdf/mirrorBsdf.hpp"
#include "math/mathUtil.hpp"
#include "bsdf/bsdfSample.hpp"

namespace pathtracer{

    MirrorBsdf::MirrorBsdf(real reflectance) : 
        m_reflectance(reflectance){}


    BsdfSample MirrorBsdf::sample(const Vector3& wo, const Vector2& uv) 
        const {

        // We only scatter light in the upper hemisphere.
        if(ShadingSpace::cosineTheta(wo) <= 0) {
            return BsdfSample::INVALID;
        }

        // Because this is a perfect mirror, we only need to
        // sample a single wi = wo reflected, which has a pdf of
        // infinity of being chosen, effectively meaning 
        // the integral is removed.

        Vector3 direction = ShadingSpace::reflect(wo);

        // The idea is that it does not contribute in any way,
        // so we'll place 1 as a placeholder.
        real pdf = 1.0;

        real cosine = ShadingSpace::cosineTheta(direction);

        Vector3 bsdf = m_reflectance / cosine;
        Vector3 weight = m_reflectance;

        return BsdfSample(bsdf, direction, cosine, pdf, weight, true);
    }


    BsdfSample MirrorBsdf::evaluate(const Vector3& wo, 
        const Vector3& wi, const Vector2& uv) const {

        // The probability that the given wi is the reflection of wo
        // is 0, so we return an invalid sample.
        return BsdfSample::INVALID;
    }


    bool MirrorBsdf::isSpecular() const{
        return true;
    }


    Vector3 MirrorBsdf::evaluateReflection(const Vector3& wo,
        const Vector2& uv) const {

        Vector3 wi = ShadingSpace::reflect(wo);
        real cosine = ShadingSpace::cosineTheta(wi);
        Vector3 bsdf = m_reflectance / cosine;

        return bsdf;
    }


    Vector3 MirrorBsdf::evaluateRefraction(const Vector3& wo, 
        const Vector2& uv) const {

        return Vector3(0.0);
    }


    real MirrorBsdf::eta(const Vector3& wo, const Vector2& uv) const {
        return 1.0;
    }

}