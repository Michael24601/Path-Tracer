
#ifndef PATH_TRACER_PIXEL_INTEGRATOR_GUIDED_HPP
#define PATH_TRACER_PIXEL_INTEGRATOR_GUIDED_HPP

#include "pixelIntegrator.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    class KdTree;
    class SurfacePoint;
    class Scene;
    class Camera;
    class ProgressReporter;


    // The guided (or bsdf fallback) sample
    struct GuidedSample{
        Vector3 wi;
        // Weight includes bsdf, cosine, and the pdf
        Vector3 weight;
        real pdf;
        bool isValid;
        // This is false if we use non-guided sample (like on first
        // iteration or on speculars)
        bool isGuided;
    };


    class PixelIntegratorGuided: public PixelIntegrator{

    protected:
        
        // One tree is used for guiding the other for training
        KdTree* m_guideTree;
        KdTree* m_trainTree;

        // The alpha is used as a weight for guiding MIS
        real m_alpha;

        // The number of training iterations
        int m_iterations;

        // The initial number of training samples per pixel
        int m_initialSamples;

        // The initial threshold for the kdTree
        real m_c;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};

        // These are guide and train tree functions

        // This function samples the current intersection point using alpha
        // and checking if it is specular.
        GuidedSample sampleGuided(const Vector3& wo,
            const SurfacePoint& it, const Scene& scene)const;


        // Does the same, but evaluates it using a given direction
        GuidedSample evaluateGuided(const Vector3& wo, const Vector3& wi,
            const SurfacePoint& it, const Scene& scene)const;

        // Function that transforms the scene positions into ones
        // in (0, 0, 0), (1, 1, 1).
        Vector3 toTreeLocalSpace(
            const Vector3& position, const Scene& scene) const;


        void recordPath(
            const std::vector<SurfacePoint>& position,
            const std::vector<Vector3>& wi,
            const std::vector<Vector3>& weight,
            const std::vector<real>& pdf,
            const std::vector<Vector3>& contribution,
            const Vector3& finalEmission,
            const Scene& scene
        );


        // Records one vertex and one direction, used for NEE samples,
        // and other similar methods.
        // Contribution must include bsdf weight and cosine already.
        void recordVertex(
            const SurfacePoint& position,
            const Vector3& wi,
            const Vector3& contribution,
            const Scene& scene
        );


    public:

    
        PixelIntegratorGuided(int samples, real alpha,
            int initialSamples, int iterations, real c) :
            PixelIntegrator(samples), m_alpha{alpha},
            m_guideTree{nullptr}, m_trainTree{nullptr}, m_c{c},
            m_iterations{iterations}, m_initialSamples{initialSamples}{

            assert(alpha > 0.0 && "MIS weight must be positive");
        }


        ~PixelIntegratorGuided() override;


        // However, we will overwrite render() function in order to
        // include the training phase.
        std::vector<std::vector<Vector3>> render(const Scene&, 
            const Camera&, int width, int height,
            ProgressReporter&) override;        
            
            
        // We don't overwrite print statistics or color,
        // or jitter... as those can be overriden by the non-virtual 
        // subclasses that need guiding.


    };

}

#endif