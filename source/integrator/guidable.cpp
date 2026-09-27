
#include "integrator/guidable.hpp"
#include "pathGuiding/kdTree.hpp"
#include <cassert>
#include "core/scene.hpp"
#include "math/vector3.hpp"


namespace pathtracer{


    Guidable::Guidable() :
        m_guideTree{nullptr},
        m_trainTree{nullptr} {}


    void Guidable::setGuideTree(KdTree* tree) {
        m_guideTree = tree;
    }


    void Guidable::setTrainTree(KdTree* tree) {
        m_trainTree = tree;
    }


    const KdTree* Guidable::getGuideTree() const {
        return m_guideTree;
    }


    KdTree* Guidable::getTrainTree() const {
        return m_trainTree;
    }


    Vector3 Guidable::toTreeLocalSpace(
        const Vector3& position, const Scene& scene) const {

        Vector3 min =
            scene.getBoundingBox().minCorner() - Vector3(0.1);

        Vector3 size =
            scene.getBoundingBox().maxCorner() + Vector3(0.1) - min;

        Vector3 result = (position - min) / size;
        return result;
    }


    void Guidable::recordPath(
        const std::vector<Vector3>& position,
        const std::vector<Vector3>& wi,
        const std::vector<Vector3>& weight,
        const Vector3& finalEmission,
        const Scene& scene) {

        assert(position.size() == wi.size());
        assert(position.size() == weight.size());

        if(m_trainTree){

            Vector3 intensity = finalEmission;

            for(int i = position.size()-1; i >= 0; i--){

                Vector3 localPoint =
                    toTreeLocalSpace(position[i], scene);

                Vector3 direction = wi[i];

                m_trainTree->accumulate(
                    localPoint,
                    direction,
                    intensity.luminance()
                );

                intensity = intensity * weight[i];
            }
        }
    }

}