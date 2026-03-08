    
#ifndef PATH_TRACER_MONTE_CARLO_ESTIMATOR_HPP
#define PATH_TRACER_MONTE_CARLO_ESTIMATOR_HPP

#include "../intersection/intersection.hpp"
#include "../core/scene.hpp"

namespace pathtracer{

    class MonteCarloEstimator{

    private: 

        // The incoming radiance
        Vector3 m_radiance;

        // The weigth includes terms that multiply the radiance,
        // like the cosine, brdf, pdf, etc...
        Vector3 m_weight;

        // The incoming light direction
        Vector3 m_wi;

        // The point from where light comes
        SurfacePoint m_it;

        // The probability, in solid angles, of hitting this point
        real m_pdf;


        // Distance to the newly sampled point.
        real m_distance;

        // If the sampled point is visible
        bool m_visible;

    public:
    

        MonteCarloEstimator() : m_radiance(Vector3::ORIGIN),
            m_weight(Vector3::ORIGIN), m_wi(Vector3::ORIGIN),
            m_pdf{0.0}, m_it(SurfacePoint()), m_visible{false}, 
            m_distance{0.0}{}


        MonteCarloEstimator(const Vector3& radiance,
            const Vector3& weight, const Vector3& wi,
            const SurfacePoint& it, real pdf, real distance, bool visible) : 
            m_radiance(radiance), m_weight(weight), m_wi(wi), 
            m_pdf{pdf}, m_it{it}, m_visible{visible}, m_distance{distance} {}


        const Vector3& radiance() const { return m_radiance; }


        const Vector3& weight() const { return m_weight; }


        const Vector3& wi() const { return m_wi; }


        const SurfacePoint& it() const{ return m_it; }


        real pdf() const { return m_pdf; }

        
        bool visible() const { return m_visible; }


        real distance() const { return m_distance; }

    };


    class MonteCarloLightSample{

    private:

        // The incoming radiance
        Vector3 m_radiance;

        // The probability, in solid angles, of hitting this point
        real m_pdf;

        // The weight (cosine, bsdf) at the point that receives the
        // radiance.
        Vector3 m_weight;

        const Light* m_light;

        Vector3 m_position;

        // If the sampled point is visible
        bool m_visible;

    public:

        MonteCarloLightSample() : m_radiance(Vector3::ORIGIN),
            m_pdf{0.0}, m_visible{false}, m_weight(Vector3::ORIGIN),
            m_light{nullptr}, m_position(Vector3::ORIGIN){}


        MonteCarloLightSample(const Vector3& radiance, const Vector3& weight,
            const Vector3& position, const Light* light, real pdf, 
            bool visible) : 
            m_radiance(radiance), m_light{light}, m_position(position),
            m_pdf{pdf}, m_visible{visible}, m_weight{weight} {}


        const Vector3& radiance() const { return m_radiance; }


        const Vector3& weight() const { return m_weight; }


        const Vector3& position() const { return m_position; }


        real pdf() const { return m_pdf; }

        
        bool visible() const { return m_visible; }


        const Light* light() const { return m_light; }
    };

}

#endif