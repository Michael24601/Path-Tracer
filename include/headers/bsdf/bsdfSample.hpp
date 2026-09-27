
#ifndef PATH_TRACER_BSDF_SAMPLE_HPP
#define PATH_TRACER_BSDF_SAMPLE_HPP

#include "math/vector3.hpp"

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
            real cosine, real pdf, const Vector3& weight, bool isDelta = false);


        BsdfSample();


        bool isInvalid() const;

        
        real cosine() const;


        const Vector3& bsdf() const;

                
        const Vector3& weight() const;


        real pdf() const;


        bool isDelta() const;

        
        const Vector3& wi() const;


        void setWi(const Vector3& wi);

        
        void setCosine(real cosine);


        void setWeight(const Vector3& weight);

        
        void setPdf(real pdf);

    };

}

#endif