    
#ifndef PATH_ESTIMATOR_HPP
#define PATH_ESTIMATOR_HPP

#include "../intersection/intersection.hpp"
#include "../core/scene.hpp"

namespace pathtracer{

    class SolidAngleEstimator{

    private: 

        Vector3 m_radiance;

        // The weigth includes terms that multiply the radiance,
        // like the cosine, brdf, pdf, etc...
        Vector3 m_weight;

        // The incoming light direction
        Vector3 m_wi;

        // The probability, in solid angles, of hitting this point
        real m_pdf;

    public:

        SolidAngleEstimator() : m_radiance(Vector3::ORIGIN),
            m_weight(Vector3::ORIGIN), m_wi(Vector3::ORIGIN),
            m_pdf{0.0}{}


        SolidAngleEstimator(const Vector3& radiance,
            const Vector3& weight, const Vector3& wi, real pdf) : 
            m_radiance(radiance), m_weight(weight), m_wi(wi), 
            m_pdf{pdf} {}


        const Vector3& radiance() const { return m_radiance; }


        const Vector3& weight() const { return m_weight; }


        const Vector3& wi() const { return m_wi; }


        real pdf() const { return m_pdf; }

    };


    
    // Same idea but with area estimator
    class AreaEstimator{

    private: 

        Vector3 m_radiance;

        // The weigth includes terms that multiply the radiance,
        // like the cosine, brdf, pdf, etc...
        Vector3 m_weight;

        // The next point of light as sampled by the area sample
        SurfacePoint m_point;

        // The probability, in solid angles, of hitting this point
        // (It's written in solid angles because we need a common
        // ground pdf measure for MIS)
        real m_pdf;

        bool m_visible;

    public:

        AreaEstimator(): m_radiance(Vector3::ORIGIN),
            m_weight(Vector3::ORIGIN),
            m_point(Vector3::ORIGIN, Vector3::ORIGIN, Vector3::ORIGIN, 
                Vector3::ORIGIN, Vector2::ORIGIN, nullptr),
            m_pdf(0.0),
            m_visible(false){}

 
        AreaEstimator(const Vector3& radiance,
            const Vector3& weight,
            const SurfacePoint& point,
            real pdf,
            bool visible) : 
            m_radiance(radiance),
            m_weight(weight),
            m_point(point),
            m_pdf(pdf),
            m_visible(visible) {}


        const Vector3& radiance() const { return m_radiance; }


        const Vector3& weight() const { return m_weight; }


        const SurfacePoint& point() const { return m_point; }


        real pdf() const { return m_pdf; }


        bool visible() const { return m_visible; }


        void setVisible(bool v) { m_visible = v; }

    };

}

#endif