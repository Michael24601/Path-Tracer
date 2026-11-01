
#ifndef PATH_TRACER_SPECULAR_MANIFOLD_SAMPLING_HPP
#define PATH_TRACER_SPECULAR_MANIFOLD_SAMPLING_HPP

#include <atomic>
#include <vector>
#include <string>
#include "config.hpp"

namespace pathtracer{

    class SurfacePoint;
    class Instance;
    class SpecularManifoldBsdf;
    class Scene;
    class Vector3;
    class SmsSample;
    class LightSample;


    // Single scatter event SMS
    class SpecularManifoldSampling{

    private:

        // Keeps a pointer to specular instances
        std::vector<const Instance*> m_specularInstances;

        // In order to avoid repeated downcasting calls,
        // the specular bsdfs are cached here (instance holds pointer
        // to base class).
        // The downcast is necessary for specular specific functions.
        std::vector<const SpecularManifoldBsdf*> m_bsdfs;

        // Maximum numbers of allowed iterations for the newton solver
        int m_maxIterations;

        // Maximum trials allowed to compute inverse probability
        int m_maxTrials;

        // Same, but for the biased version
        int m_maxBiasedTrials;

        // The threshold for two samples being the same
        real m_threshold;

        // The threshold for checking if the specular constraint is met.
        real m_epsilon;

        // Uses half vector constraint instead of angle difference
        bool m_useHalfVector;

        // Uses biased sms
        bool m_biased;

        bool m_twoStage;

    public:

        // Debug variables
        std::atomic<long> smoothRun{0};
        std::atomic<long> smoothConverged{0};
        std::atomic<long> smoothIterationSum{0};
        std::atomic<long> trueRun{0};
        std::atomic<long> trueConverged{0};
        std::atomic<long> trueIterationSum{0};
        std::atomic<long> unbiasedSuccess{0};
        std::atomic<long> counterSum{0};
        std::atomic<real> averageUniquePerRun{0.0};
        std::atomic<long> contributingSamples{0};
        std::atomic<long> zeroGeometricTerm{0};

        SpecularManifoldSampling(const Scene* scene, bool useHalfVector, 
            bool biased, bool twoStage, int maxIterations = 30, 
            int maxTrials = 1000, int maxBiasedTrials = 10, 
            real epsilon = 1e-9, real threshold = 1e-6);


        // Returns the outgoing radiance from x0, after connecting
        // x0 to a specular point x1, connected to x2 on the light, 
        // satisfying the specular constraint.
        // The wo pointing away from x0 to the previous point is also sent.
        Vector3 sample(const SurfacePoint& causticPoint, 
            const Vector3& wo, const Scene* scene);


        // Samples a path and computes the contribution without bias.
        // This includes all terms except the light selection pdf which sample
        // handles.
        Vector3 unbiasedSample(const SurfacePoint& causticPoint,
            const Instance* specular, const SpecularManifoldBsdf* bsdf,
            const LightSample& lightPoint, const Vector3& wo, 
            const Vector3& localOffsetNormal, const Scene* scene);


        // Same but biased (less variance).
        Vector3 biasedSample(const SurfacePoint& causticPoint,
            const Instance* specular, const SpecularManifoldBsdf* bsdf,
            const LightSample& lightPoint, const Vector3& wo, 
            const Vector3& localOffsetNormal, const Scene* scene);

            
        // Given a point on a light x2 (that we already 
        // supposedly sampled outside), a point on a surface received a 
        // caustic x0, and a specular instance in between, this function
        // constructs a path through the specular surface by finding a
        // point x1 on it that satisfies the specular constraint.
        // It returns the seed point, and the converged point (after newton's
        // method is used to satisfy the constraints). May return
        // a non converged result.
        SmsSample samplePath(const SurfacePoint& causticPoint, 
            const Instance* specular, const SpecularManifoldBsdf* bsdf, 
            const LightSample& lightPoint, const Vector3& localOffsetNormal,
            const Scene* scene);


        SmsSample newtonSolver(const SurfacePoint& causticPoint, 
            const SurfacePoint& seedIt, const LightSample& lightPoint, 
            const SpecularManifoldBsdf* bsdf, const Vector3& localOffsetNormal,
            const Scene* scene, bool smooth);


        // Returns the ougoing contribution from x0 after it samples
        // x1 and x2 on the light. This will include all weight terms
        // except for the pdf at x1, which is computed outside
        // (since it can fail, so the main function handles it),
        // and the light selection pdf.
        Vector3 evaluatePathContribution(
            const SurfacePoint& causticPoint, const LightSample& lightPoint, 
            const SmsSample& sample, const SpecularManifoldBsdf* bsdf, 
            const Vector3& wo, const Scene* scene, const Vector3& lon);


        std::string toString() const{

            std::string info;

            info += "SMS ";
            info += std::string(m_useHalfVector ? "HV " : "AD ");
            info += std::string(m_biased ? "BIASED " : "UNBIASED ");
            info += std::string(m_twoStage ? "TWO_STAGE" : "ONE_STAGE");

            return info;
        }

    };

}

#endif