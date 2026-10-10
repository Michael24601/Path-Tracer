#ifndef PATH_TRACER_GUIDABLE_HPP
#define PATH_TRACER_GUIDABLE_HPP

#include <atomic>
#include <vector>
#include "config.hpp"

namespace pathtracer{

    class Scene;
    class Vector3;
    class KdTree;

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

        const KdTree* getGuideTree() const;

        KdTree* getTrainTree() const;

        // Function that transforms the scene positions into ones
        // in (0, 0, 0), (1, 1, 1).
        Vector3 toTreeLocalSpace(
            const Vector3& position, const Scene& scene) const;


        void recordPath(
            const std::vector<Vector3>& position,
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
            const Vector3& position,
            const Vector3& wi,
            const Vector3& contribution,
            const Scene& scene
        );
    };

}

#endif