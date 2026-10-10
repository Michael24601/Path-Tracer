
#include "integrator/guidable.hpp"
#include "pathGuiding/kdTree.hpp"
#include <cassert>
#include "core/scene.hpp"
#include "math/vector3.hpp"
#include "logger.hpp"


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


    void Guidable::setAlpha(real alpha){
        m_alpha = alpha;
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
        const std::vector<real>& pdf,
        const std::vector<Vector3>& contribution,
        const Vector3& finalEmission,
        const Scene& scene) {

        assert(position.size() == wi.size());
        assert(position.size() == weight.size());
        assert(position.size() == pdf.size());

        if(m_trainTree){

            Vector3 intensity = finalEmission;

            for(int i = position.size()-1; i >= 0; i--){

                Vector3 localPoint = toTreeLocalSpace(position[i], scene);

                Vector3 direction = wi[i];

                // In accumulate, we add the incoming radiance from wi to
                // current vertex, divided by the pdf of the direction wi.
                // We DO NOT include the bsdf and the cosine because they
                // are unique to the wo direction during training, and will
                // wrongly skew the result during rendering when the wo
                // direction at the same point x is different.
                Vector3 incoming = intensity / pdf[i];

                m_trainTree->accumulate(localPoint, direction,
                    incoming.mean());


                // The incoming direction for the next iteration then
                // multiplies by the full weight with the pdf, cosine,
                // and bsdf. It also adds any other contributions (like NEE)
                // at x now (could not add them to the x quadtree node
                // since they come from another direction than wi).
                intensity = intensity * weight[i] + contribution[i];

            }
        }
    }


    void Guidable::recordVertex(const Vector3& position,
        const Vector3& wi, const Vector3& contribution, const Scene& scene){

        if(m_trainTree){
            Vector3 localPoint = toTreeLocalSpace(position, scene);
            m_trainTree->accumulate(localPoint, wi, contribution.mean());
        }
        
    }

}