#ifndef PATH_TRACER_GUIDABLE_HPP
#define PATH_TRACER_GUIDABLE_HPP

#include <atomic>
#include <vector>

namespace pathtracer{

    class Scene;
    class Vector3;
    class KdTree;

    // An interface that any integrator that needs to use path guiding
    // must implement to access trees.
    class Guidable{

    protected:

        KdTree* m_guideTree;
        KdTree* m_trainTree;

    public:

        std::atomic<int> pathCount;

        Guidable();

        void setGuideTree(KdTree* tree);

        void setTrainTree(KdTree* tree);

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
            const Vector3& finalEmission,
            const Scene& scene);
    };

}

#endif