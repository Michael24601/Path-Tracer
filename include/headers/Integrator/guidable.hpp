
#ifndef PATH_TRACER_GUIDABLE_HPP
#define PATH_TRACER_GUIDABLE_HPP

#include "../pathGuiding/kdTree.hpp"

namespace pathtracer{

    // An interface that any integrator that needs to use path guiding
    // must implement to access trees.
    class Guidable{

    protected:

        KdTree* m_guideTree;
        KdTree* m_trainTree;

    public:
    
        std::atomic<int> pathCount;

        Guidable() :
            m_guideTree{nullptr},
            m_trainTree{nullptr} {}

        void setGuideTree(KdTree* tree) {
            m_guideTree = tree;
        }

        void setTrainTree(KdTree* tree){
            m_trainTree = tree;
        }

        const KdTree* getGuideTree() const {
            return m_guideTree;
        }

        KdTree* getTrainTree() const {
            return m_trainTree;
        }


        // Function that transforms the scene positions into ones
        // in (0, 0, 0), (1, 1, 1).
        Vector3 toTreeLocalSpace(const Vector3& position, const Scene& scene) const{
            Vector3 min = scene.getBoundingBox().minCorner() - Vector3(0.1);
            Vector3 size = scene.getBoundingBox().maxCorner() + Vector3(0.1) - min;

            Vector3 result = (position - min) / size;
            return result;
        }


        void recordPath(const std::vector<Vector3>& position,
            const std::vector<Vector3>& wi, 
            const std::vector<Vector3>& weight,
            const Vector3& finalEmission, const Scene& scene){

            assert(position.size() == wi.size());
            assert(position.size() == weight.size());

            if(m_trainTree){

                Vector3 intensity = finalEmission;

                for(int i = position.size()-1; i >= 0; i--){

                    Vector3 localPoint = toTreeLocalSpace(position[i], scene);
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

    };

}

#endif