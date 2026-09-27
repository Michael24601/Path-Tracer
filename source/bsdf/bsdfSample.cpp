#include "bsdf/bsdfSample.hpp"

namespace pathtracer{
    

    BsdfSample BsdfSample::INVALID = 
        BsdfSample(Vector3(0.0), Vector3(0.0), 0.0, 0.0, Vector3(0.0));


    BsdfSample::BsdfSample(const Vector3& bsdf, const Vector3& wi, 
        real cosine, real pdf, const Vector3& weight, bool isDelta) : 
        m_cosine{cosine}, m_bsdf{bsdf},  
        m_wi(wi), m_pdf{pdf}, m_weight{weight}, m_isDelta{isDelta}{}


    BsdfSample::BsdfSample() : 
        m_bsdf{Vector3(0.0)}, m_wi{Vector3(0.0)}, 
        m_cosine{0.0}, m_pdf{0.0}, m_weight{Vector3(0.0)}, 
        m_isDelta{false}{}


    bool BsdfSample::isInvalid() const { 
        return m_pdf <= 0.0; 
    }


    real BsdfSample::cosine() const { 
        return m_cosine; 
    }


    const Vector3& BsdfSample::bsdf() const { 
        return m_bsdf; 
    }


    const Vector3& BsdfSample::weight() const { 
        return m_weight; 
    }


    real BsdfSample::pdf() const { 
        return m_pdf; 
    }


    bool BsdfSample::isDelta() const { 
        return m_isDelta; 
    }


    const Vector3& BsdfSample::wi() const { 
        return m_wi; 
    }


    void BsdfSample::setWi(const Vector3& wi) { 
        m_wi = wi; 
    }


    void BsdfSample::setCosine(real cosine) { 
        m_cosine = cosine; 
    }


    void BsdfSample::setWeight(const Vector3& weight) { 
        m_weight = weight; 
    }


    void BsdfSample::setPdf(real pdf) { 
        m_pdf = pdf; 
    }

}