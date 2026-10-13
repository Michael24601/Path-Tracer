#ifndef PATH_TRACER_GUIDABLE_HPP
#define PATH_TRACER_GUIDABLE_HPP

#include <atomic>
#include <vector>
#include "math/vector3.hpp"

namespace pathtracer{

    class Scene;
    class KdTree;
    class SurfacePoint;


    // The guided (or bsdf fallback) sample
    struct GuidedSample{
        Vector3 wi;
        // Weight includes bsdf, cosine, and the pdf
        Vector3 weight;
        real pdf;
        bool isValid;
    };


    // An interface that any integrator that needs to use path guiding
    // must implement to access trees.
    class Guidable{

    protected:

    public:

        KdTree* m_guideTree;
        KdTree* m_trainTree;

        real m_alpha;

        std::atomic<int> pathCount;

        Guidable();

        void setGuideTree(KdTree* tree);

        void setTrainTree(KdTree* tree);

        void setAlpha(real alpha);

        real alpha() const;

        // This function samples the current intersection point using alpha
        // and checking if it is specular.
        GuidedSample sampleGuided(const Vector3& wo,
            const SurfacePoint& it, const Scene& scene)const;


        // Does the same, but evaluates it using a given direction
        GuidedSample evaluateGuided(const Vector3& wo, const Vector3& wi,
            const SurfacePoint& it, const Scene& scene)const;

        const KdTree* getGuideTree() const;

        KdTree* getTrainTree() const;

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
    };

}

#endif