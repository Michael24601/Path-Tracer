
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
            Vector3 min = scene.getBoundingBox().minCorner();
            Vector3 size = scene.getBoundingBox().maxCorner() - min;

            return (position - min) / size;
        }



    };

}

#endif