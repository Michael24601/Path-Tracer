
#ifndef PATH_TRACER_BSDF_SAMPLE_HPP
#define PATH_TRACER_BSDF_SAMPLE_HPP

#include "../core/ray.hpp"

namespace pathtracer{

    class BsdfSample{

        // The term cos(theta_i)
        real m_cosine;

        // The bsdf term fr(x, wi, wo)
        Vector3 m_bsdf;

        // The sample direction wi
        Vector3 m_wi;

        // The probability density of choosing the direction
        real m_pdf;

        // The weight term (cosine * bsdf / pdf)
        // Stored separately since BSDF may avoid redundant calculations
        // like for cosine weighted sampling.
        Vector3 m_weight;


        bool m_isDelta;

    public:

        static BsdfSample INVALID;
        
        
        BsdfSample(const Vector3& bsdf, const Vector3& wi, 
            real cosine, real pdf, const Vector3& weight, bool isDelta = false): 
            m_cosine{cosine}, m_bsdf{bsdf},  
            m_wi(wi), m_pdf{pdf}, m_weight{weight}, m_isDelta{isDelta}{}


        bool isInvalid() const { return m_pdf <= 0.0; }

        
        real cosine() const { return m_cosine; }


        const Vector3& bsdf() const { return m_bsdf; }

                
        const Vector3& weight() const { return m_weight; }


        real pdf() const { return m_pdf; }


        bool isDelta() const { return m_isDelta; }

        
        const Vector3& wi() const { return m_wi; }


        void setWi(const Vector3& wi) { m_wi = wi; }

        
        void setCosine(real cosine) { m_cosine = cosine; }


        void setWeight(const Vector3& weight) { m_weight = weight; }

        
        void setPdf(real pdf) { m_pdf = pdf; }

    };

}

#endif