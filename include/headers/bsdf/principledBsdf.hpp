
#ifndef PATH_TRACER_PRINCIPLED_BSDF_HPP
#define PATH_TRACER_PRINCIPLED_BSDF_HPP

#include "bsdf.hpp"
#include "../math/mathUtil.hpp"
#include "../core/random.hpp"
#include "../texture/texture.hpp"
#include "microfacetUtil.hpp"
#include "fresnelUtil.hpp"

namespace pathtracer{

    // Not a bsdf subclass, just a helper
    class DiffuseLobe {
    
    private:

        // We don't need to store a texture here since this is not a bsdf.
        // The texture is sampled in the bsdf and the color for one pixel
        // is sent here.
        Vector3 m_color;

    public:


        DiffuseLobe(const Vector3& color) : m_color{color}{}


        BsdfSample evaluate(const Vector3 &wo, const Vector3 &wi) const {

            Vector3 weight;
            Vector3 bsdf;
            real pdf;
            real cosine;

            // Note that the input and output directions should both
            // be on the same side of the surface.
            if(wo.z() * wi.z() <= 0) {
                weight = bsdf = Vector3(0.0f);
                pdf = 0.0;
                cosine = 0.0;
            }
            else{
                pdf = SquareToHemisphereCosine::pdf(wi);
                cosine = Bsdf::cosineTheta(wi);
                Vector3 albedo = m_color;
                bsdf = albedo * INV_PI;
                // Cosine cancels out when using cosine weighted sampling 
                // (includes cosine, bsdf, and pdf terms)
                weight = albedo;
            }

            return BsdfSample(bsdf, wi, cosine, pdf, weight);
        }


        BsdfSample sample(const Vector3 &wo) const {

            Vector3 wi = SquareToHemisphereCosine::transform(Random::next2D());
            wi = wi.normalized();

            // The weight of a sample is basically all 3 terms multiplying
            // Le, and those are fr, |cos(theta)|, and 1/pdf.
            // Note that since we are in local coordinates, the normal
            // vector is just (0, 0, 1).
            // However, the cos term cancels out along with the pdf.
            // Note that the invPi also cancels out because the density
            // is cosine / pi.
            // This leaves only the fr(x, wo, wi) value.
            Vector3 albedo = m_color;
            Vector3 weight = albedo;
            Vector3 bsdf = albedo * INV_PI;

            if (Bsdf::cosineTheta(wo) < 0){
                wi = -wi; 
            }

            real pdf = SquareToHemisphereCosine::pdf(wi);
            real cosine = Bsdf::cosineTheta(wi);

            return BsdfSample(bsdf, wi, cosine, pdf, weight);
        }


        const Vector3&  color() const{
            return m_color;
        }

    };

    struct MetallicLobe {

    private:

        real m_alpha;
        
        // We don't need to store a texture here since this is not a bsdf.
        // The texture is sampled in the bsdf and the color for one pixel
        // is sent here.
        Vector3 m_color;

    public:


        MetallicLobe(const Vector3& color, real alpha) : m_color{color},
            m_alpha{alpha}{}


        BsdfSample evaluate(const Vector3 &wo, const Vector3 &wi) const {

            // This here is the microfacet evaluate function, which
            // just evaluates the microfacet BRDF using:
            // fr(x, wi, wo) = [ R D(wm) G1(wi) G1(wo) ] / [ 4 |cos_wi| |cos_wo| ]
            // We have access to all these terms except wm, which is the
            // halfway vector.
            Vector3 wm = (wi + wo) * (1.0 / (wi + wo).length());
            
            // I used the normal here instead of wm for the reflection,
            // since we need cos(thetao). Doesn't worl with wm.
            real cos_wo = fabs(wo.z());
            Vector3 R = m_color;
            real G1_wi = Microfacet::smithG1(m_alpha, wm, wi);
            real G1_wo = Microfacet::smithG1(m_alpha, wm, wo);
            real D_wm = Microfacet::evaluateGGX(m_alpha, wm);

            // PDF
            real pdf = Microfacet::pdfGGXVNDF(m_alpha, wm, wo);
            real cosine = Bsdf::cosineTheta(wi);
            Vector3 bsdf = (R * D_wm * G1_wi * G1_wo) * (1.0 / (4 * cos_wo));

            Vector3 weight = bsdf * cosine * (1.0 / pdf);

            return  BsdfSample (bsdf, wi, cosine, pdf, weight);
        }


        BsdfSample sample(const Vector3 &wo) const {

            // We have to sample a direction wi and assign it a weight.
            // As we want a reflective but rough surface, we will sample
            // the halfway vector from a distribution with density p(wm),
            // and then use that to calculate wi by reflecting it around it.
            Vector3 wm = Microfacet::sampleGGXVNDF(m_alpha, wo, Random::next2D());
            // The density function will be pdfGGXVNDF.

            // Formula for vector wi reflected around wm.
            Vector3 wi = Bsdf::reflect(wo, wm);
            wi = wi.normalized();
            // This will also have a pdf, which scales p(wm) by 1/(4 |wm.wo|)
            // And p(wm) is [ D(wm) G1_wo |wm . wo| ] / |cos_wo|

            // Now for the weight, we know that the weight is usually:
            // fr(x, wo, wi) * |cos_wi| * 1/pdf.
            // This is equivalent to:
            // [ R * D_wm * G1_wi * G1_wo * |cos_wi| * 4 * |wm . wo| * |cos_wo| ] 
            // ----------------------------------------
            // [ 4 * |cos_wi| * |cos_wo| * D_wm * G1_wo * |wm . wo| ]
            // This leaves only: R * G1_wi
            
            // There is no need to check if(wo.z() * wi.z() > 0) 
            // since this will be the case by default (I'm assuming).

            Vector3 R = m_color;
            real G1_wi = Microfacet::smithG1(m_alpha, wm, wi);
            Vector3 weight = R * G1_wi;

            real cosine = Bsdf::absCosineTheta(wi);

            real pdf = Microfacet::pdfGGXVNDF(m_alpha, wm, wo);

            real D_wm = Microfacet::evaluateGGX(m_alpha, wm);
            real G1_wo = Microfacet::smithG1(m_alpha, wm, wo);
            real cos_wo = Bsdf::absCosineTheta(wo);
            Vector3 bsdf = (R * D_wm * G1_wi * G1_wo) * (1.0 / (4 * cos_wo));

            return BsdfSample(bsdf, wi, cosine, pdf, weight);
        }


        const Vector3&  color() const{
            return m_color;
        }


        real alpha() const{
            return m_alpha;
        }

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
                const MetallicLobe& metallic
            ): diffuseSelectionProb(diffuseSelectionProb),
                diffuse(diffuse),
                metallic(metallic) {}

        };

        Combination combine(const Vector2 &uv, const Vector3& wo) const {
            const auto baseColor = m_baseColor->sample(uv);
            // Assuming it is greyscale
            const real roughness = m_roughness->sample(uv).x();
            const real alpha = std::max(real(1e-3), roughness * roughness);
            const real specular = m_specular->sample(uv).x();
            const real metallic = m_metallic->sample(uv).x();
            const real F = specular * Fresnel::schlick((1 - metallic) 
                * 0.08f, Bsdf::cosineTheta(wo));

            const DiffuseLobe diffuseLobe(baseColor * (1 - F) * (1 - metallic));
            const MetallicLobe metallicLobe(baseColor * F + (1 - F) * metallic, alpha);

            const real diffuseAlbedo = diffuseLobe.color().mean();
            const real totalAlbedo =
                diffuseLobe.color().mean() + metallicLobe.color().mean();

            return Combination(
                (totalAlbedo > 0 ? diffuseAlbedo / totalAlbedo : 1.0),
                diffuseLobe,
                metallicLobe
            );
        }


    public:


        PrincipledBsdf(Texture* baseColor, Texture* roughness,
            Texture* metallic, Texture* specular) : 
            m_baseColor(baseColor),
            m_roughness(roughness),
            m_metallic(metallic),
            m_specular(specular) {}


        BsdfSample evaluate(const Vector3 &wo, const Vector3 &wi,
            const Vector2& uv) const override {

            const Combination combination = combine(uv, wo);

            // We just compute the two and sum them
            BsdfSample diffuse_component = combination.diffuse.evaluate(wo, wi);
            BsdfSample metallic_component = combination.metallic.evaluate(wo, wi);

            // I chose the pdf that combines the two using the probability.
            real p = combination.diffuseSelectionProb;

            // TODO: CHECK
            Vector3 bsdf = diffuse_component.bsdf() + metallic_component.bsdf();

            real pdf = diffuse_component.pdf() * p + metallic_component.pdf() * (1 - p);

            real cosine = Bsdf::absCosineTheta(wi);

            Vector3 weight = (diffuse_component.weight() * diffuse_component.pdf() 
                + metallic_component.weight() * metallic_component.pdf())
                * (1.0 / pdf);

            return BsdfSample(bsdf, wi, cosine, pdf, weight);
        }

        
        BsdfSample sample(const Vector3 &wo, const Vector2& uv) const override {

            const auto combination = combine(uv, wo);
            
            real p = combination.diffuseSelectionProb;
            real random = Random::next();
            
            if(random < p){
                BsdfSample sample = combination.diffuse.sample(wo);
                // Updates the weight and pdf since the pdf now also
                // gets multiplied by p (weigth includes 1/pdf).
                sample.setWeight(sample.weight() * (1.0 / p));
                sample.setPdf(sample.pdf() * p);
                return sample;
            }
            else{
                BsdfSample sample = combination.metallic.sample(wo);
                sample.setWeight(sample.weight() * (1.0 / (1.0-p)));
                sample.setPdf(sample.pdf() * (1.0 - p));
                return sample;
            }
        }


        bool isSpecular() const override{
            return false;
        }

    };

}

#endif