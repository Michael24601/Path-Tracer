
#include "bsdf/roughConductorBsdf.hpp"
#include "math/mathUtil.hpp"
#include "bsdf/bsdfSample.hpp"
#include "texture/texture.hpp"
#include "bsdf/microfacetUtil.hpp"
#include "core/random.hpp"
#include "math/constants.hpp"
#include "math/vector2.hpp"

namespace pathtracer{


    RoughConductorBsdf::RoughConductorBsdf(Texture* reflectance,
        Texture* roughness) : m_reflectance(reflectance), 
        m_roughness(roughness){}


    BsdfSample RoughConductorBsdf::sample(const Vector3& wo, const Vector2& uv,
        bool adjoint) const {

        const real roughness = m_roughness->sample(uv).x();
        const real alpha = std::max(real(1e-3), sqrtReal(roughness));

        Vector3 wm = Microfacet::sampleGGXVNDF(alpha, wo, Random::next2D());
        Vector3 wi = ShadingSpace::reflect(wo, wm).normalized();

        if (wo.z() * wi.z() <= 0.0) {
            return BsdfSample();
        }

        // Now for the weight, we know that the weight is usually:
        // fr(x, wo, wi) * |cos_wi| * 1/pdf.
        // This is equivalent to:
        // [ R * D_wm * G1_wi * G1_wo * |cos_wi| * 4 * |wm . wo| * |cos_wo| ] 
        // ----------------------------------------
        // [ 4 * |cos_wi| * |cos_wo| * D_wm * G1_wo * |wm . wo| ]
        // This leaves only: R * G1_wi
        
        // There is no need to check if(wo.z() * wi.z() > 0) 
        // since this will be the case by default (I'm assuming).


        const real cosine = abs(wi.z());
        const real cosWo = abs(wo.z());
        const real G1Wi = Microfacet::smithG1(alpha, wm, wi);
        const real G1Wo = Microfacet::smithG1(alpha, wm, wo);
        const real D = Microfacet::evaluateGGX(alpha, wm);

        const Vector3 bsdf = m_reflectance->sample(uv)
            * (D * G1Wi * G1Wo / (4.0 * cosWo * cosine));

        const real pdf = Microfacet::pdfGGXVNDF(alpha, wm, wo)
            / (4.0 * abs(wm.dot(wo)));

        const Vector3 weight = m_reflectance->sample(uv)
            * Microfacet::smithG1(alpha, wm, wi);

        return BsdfSample(bsdf, wi, cosine, pdf, weight, false);
    }


    BsdfSample RoughConductorBsdf::evaluate(const Vector3& wo,
        const Vector3& wi, const Vector2& uv, bool adjoint) const {

        if (wo.z() * wi.z() <= 0.0) {
            return BsdfSample();
        }

        const real roughness = m_roughness->sample(uv).x();
        const real alpha = std::max(real(1e-3), sqrtReal(roughness));

        const Vector3 wm = (wo + wi).normalized();

        const real cosine = abs(wi.z());
        const real cosWo = abs(wo.z());

        if (cosWo == 0.0 || abs(wm.dot(wo)) == 0.0) {
            return BsdfSample();
        }

        const real G1Wi = Microfacet::smithG1(alpha, wm, wi);
        const real G1Wo = Microfacet::smithG1(alpha, wm, wo);
        const real D = Microfacet::evaluateGGX(alpha, wm);

        const Vector3 bsdf = m_reflectance->sample(uv)
            * (D * G1Wi * G1Wo / (4.0 * cosWo * cosine));

        const real pdf = Microfacet::pdfGGXVNDF(alpha, wm, wo)
            / (4.0 * abs(wm.dot(wo)));

        const Vector3 weight = m_reflectance->sample(uv) * G1Wi;

        return BsdfSample(bsdf, wi, cosine, pdf, weight, false);
    }


    // Essentially returns the weight of reflecting
    Vector3 RoughConductorBsdf::evaluateReflection(const Vector3& wo,
        const Vector3& wi, const Vector2& uv) const {

        if (wo.z() * wi.z() <= 0.0) {
            return Vector3(0.0);
        }

        const real roughness = m_roughness->sample(uv).x();
        const real alpha = std::max(real(1e-3), sqrtReal(roughness));

        const Vector3 wm = (wo + wi).normalized();

        return m_reflectance->sample(uv) * Microfacet::smithG1(alpha, wm, wi);
    }


    Vector3 RoughConductorBsdf::evaluateRefraction(const Vector3& wo, 
        const Vector3& wi, const Vector2& uv) const {

        return Vector3(0.0);
    }


    ManifoldNormal RoughConductorBsdf::sampleManifoldNormal(const Vector2& uv) const {
        const real roughness = m_roughness->sample(uv).x();
        const real alpha = std::max(real(1e-3), sqrtReal(roughness));

        const Vector2 u = Random::next2D();

        const real ux = std::min(u.x(), real(1.0) - real(1e-7));
        const real tan2Theta = alpha * alpha * ux / (real(1.0) - ux);
        const real cosTheta = real(1.0) / sqrtReal(real(1.0) + tan2Theta);
        const real sinTheta = sqrtReal(std::max(real(0.0), real(1.0) - cosTheta * cosTheta));
        const real phi = real(2.0) * PI * u.y();

        const Vector3 wm(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);

        const real pdf = Microfacet::evaluateGGX(alpha, wm) * cosTheta;

        return ManifoldNormal{wm, pdf};
    }


}