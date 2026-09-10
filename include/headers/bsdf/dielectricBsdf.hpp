
#ifndef PATH_TRACER_DIELECTRIC_BSDF_HPP
#define PATH_TRACER_DIELECTRIC_BSDF_HPP

#include "bsdf.hpp"
#include "../math/mathUtil.hpp"
#include "../core/random.hpp"
#include "../texture/texture.hpp"
#include "fresnelUtil.hpp"

namespace pathtracer
{

    class DielectricBsdf : public Bsdf{

    private:
        // The color a this sample point could either come from a texture
        // or from an albedo. It is set from outside.
        const Texture *m_reflectance;
        const Texture *m_transmittance;
        const Texture *m_ior;

    public:
        DielectricBsdf(const Texture *ior, const Texture *reflectance,
            const Texture *transmittance) : m_ior{ior},
            m_reflectance{reflectance}, m_transmittance{transmittance} {}

        BsdfSample sample(const Vector3 &wo, const Vector2 &uv)
            const override
        {

            // We will randomly pick either to refract or reflect,
            // and the probability will be weighted by the Fresnel term.
            // So we will calculate that first.
            // F = (Fp^2 + Fs^2)/2 for s and p polarized light.

            real cos_wo = Bsdf::cosineTheta(wo);

            // We know that ior is n_int/n_ext, from interior to exterior (air).
            // We will pick eta, or n, as either ior or 1/ior.
            // eta = n is awlays no/ni (outgoing over incoming).
            real ior = m_ior->sample(uv).x();
            real invIor = 1.0f / ior;

            // Depending on whether we are going from interior to
            // exterior or vice-versa.
            // We always define eta, or n, as no/ni, where no is the
            // outgoing eta, and ni is the incoming eta, so it
            // n = n_ext/n_int or n_int/n_ext depending on whether
            // we are exitting or entering.
            real n;
            if (cos_wo > 0)
            {
                // If it is negative, then we are going from exterior to
                // interior.
                // So n = no/ni = n_int/n_ext = ior
                n = ior;
            }
            else
            {
                // If it is positive, then we are going from interior to
                // exterior.
                // So n = no/ni = n_ext/n_int = 1 / ior
                n = invIor;
            }

            // According to one of the tutors, there is no need to check
            // for TIR, since it is accounted for in the probability fresnel
            // term.
            // However, if we do need to check, then we compute sin_wi,
            // and then check if(sin_wi_2 <= 1.0f).
            // You can see me doing that in the code above.

            // We have a function for Fresnel
            real fresnel = Fresnel::dielectric(cos_wo, n);
            real random = Random::next();

            // Now, we can do either reflection or refraction
            Vector3 transmittance_color = m_transmittance->sample(uv);
            Vector3 reflectance_color = m_reflectance->sample(uv);

            // We always reflect, but there may not be any
            // transmittance, which is what we need to check.
            bool reflectance = random < fresnel;
            bool transmittance = transmittance_color != Vector3(0.0);

            // The bsdf is Fr * reflectance_color,
            // or (1-Fr) * transmittance_color.
            // We however divide by the probability of picking either,
            // so the fresnel terms cancel out.
            // However, if we choose to reflect because no transmittance
            // is here, then we divide by 1, and the fresnel term
            // remains.

            // The pdf is set to 1, not because it's 1, but because it is
            // a delta distribution, so we ignore the pdf.
            real pdf = 1.0;
            bool isDelta = true;
            Vector3 bsdf;
            Vector3 weight;
            real cosineTerm;
            Vector3 wi;

            // NOTE: Frensel term here is the probability of refracting
            // or reflecting, so we need to take it into consideration
            // inside pdf and weight (which contains 1/pdf).
            // But here, the pdf comes from a delta distribution,
            // so we can't represent it numerically.
            // So we just incorporate the Fresnel and 1-Fresnel probabilities
            // in the weight as parts of the pdf reciprocal.

            if (reflectance || !transmittance){
                // Assuming the reflection is deterministic, like
                // the conductor.
                // The normal is (0, 0, 1) since we are in local space.
                Vector3 normal(0, 0, 1);
                wi = reflect(wo, normal);
                wi = wi.normalized();
                cosineTerm = Bsdf::cosineTheta(wi);
                
                // Again, as before, the pdf which is 1/0 and the dirac
                // delta cancel out, along with the cos term.
                // This leaves Fr * color / Fr, or Fr * color as
                // discussed.
                if (!transmittance) {
                    bsdf = reflectance_color * fresnel * (1.0 / cosineTerm);
                    weight = reflectance_color * fresnel;
                }
                else {
                    bsdf = reflectance_color * fresnel * (1.0 / cosineTerm);
                    weight = reflectance_color;
                }
            }
            else {
                // Now for the refraction, we know that wi is.
                Vector3 normal(0, 0, 1);
                wi = Bsdf::refract(wo, normal, n);
                wi = wi.normalized();
                cosineTerm = Bsdf::cosineTheta(wi);
                // Here eta, or n, is used in the weight.
                // Otherwise it is the same as before.
                weight = transmittance_color * (1.0 / (n * n));
                bsdf = transmittance_color *
                    (1.0f - fresnel) * (1.0 / (n * n * cosineTerm));
            }

            return BsdfSample(bsdf, wi, cosineTerm, pdf, weight, isDelta);
        }

        BsdfSample evaluate(const Vector3 &wo,
                            const Vector3 &wi, const Vector2 &uv) const override
        {

            // The probability of a light sample picking exactly the
            // direction 'wi' that results from reflecting or refracting
            // 'wo' is zero, hence we can just ignore that case and
            // always return black.
            return BsdfSample::INVALID;
        }


        bool isSpecular() const override{
            return true;
        }

    };

}

#endif