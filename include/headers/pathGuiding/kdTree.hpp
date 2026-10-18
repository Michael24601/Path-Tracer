
#ifndef PATH_TRACER_KD_TREE_HPP
#define PATH_TRACER_KD_TREE_HPP

#include <atomic>
#include <vector>
#include "config.hpp"

namespace pathtracer{

    class QuadTree;
    class Vector3;
    class AxisAlignedBox;
    enum class DirectionalFilter;
    
    enum class SpatialFilter{
        NEAREST,
        STOCHASTIC,
        BOX
    };

    // A three dimensional KdTree
    class KdTree{

    private:

        class Node;
        typedef Node* NodePtr;

        class Node{

        private:

            // The children of this node
            std::vector<NodePtr> children;

            // The sample count in a spatial node. 
            // It is atomic as multiple threads may write (sum) into it. 
            // It is a real instead of an int to allow for weighting 
            // samples differently for filters and NEE.
            std::atomic<real> sampleWeight;

            // Directional tree (only in leafs)
            QuadTree* dTree;

        public:

            Node(real weight);

            ~Node();

            // Prunes the descendants of a node
            void cleanSubtree();

            void setDTree(QuadTree* tree);

            inline bool isLeaf() const {
                return !children[0];
            }

            inline void addSample(real weight){
                sampleWeight.fetch_add(weight, std::memory_order_relaxed);
            }

            inline void setSampleCount(real weight){
                sampleWeight.store(weight);
            }

            inline real getSampleWeight() const{
                return sampleWeight.load(std::memory_order_relaxed);
            }

            // Takes a point p, which is in local coordinates of the
            // current node, and checks if it is inside.
            // The local coordinates are always the (0, 0, 0) to (1, 1, 1)
            // cube, and we split the axes alternating.
            bool inCell(const Vector3& p) const;

            // Transform a point in the local coordinate space of
            // this node to one of its children.
            Vector3 transformToChild(const Vector3& p, int child,
                int depth) const;

            // Takes point in child's local space, and transforms to
            // current node's local space.
            Vector3 transformFromChild(const Vector3& p, int child,
                int depth) const;

            // Subdivides the node, and gives each child half its sample count
            // and a copy of its quadTree.
            void subdivide();

            // This function needs to accumulate a sample. It does that
            // by going down the tree, adding the sample to the leaf
            // it arrives at, and then calling the accumulate function
            // of the dTree.
            void accumulate(const Vector3& p, const Vector3& direction, 
                real weight, real contribution, DirectionalFilter dFilter, 
                int depth);

            // Recomputes the sample count of the parent using the childrens'
            // but first recomputes children's since they may be stale too. 
            void recomputeSampleWeight();

            // Finds dTree in elaf associated with point p
            QuadTree* getDTree(const Vector3& p, int depth) const;

            // Adapts a node: if it's a leaf with too many samples
            // subdivides it into 2 children with half the count. 
            // If an internal node has too little samples, prunes its
            // descendants.
            void adaptNode(real threshold);

            // Resets the subtree (sets sample count to 0)
            // And resets all dTrees in it.
            void reset();

            // Creates a copy of the structure of the tree, with the same flux.
            // Sets the children of newNode.
            void copy(NodePtr newNode) const;


            real getSampleWeight(const Vector3& p, int depth) const;

            // Descends into leaf containing p, jitters it so it's 
            // possibly in another leaf, then returns jittered p in 
            // global root coordinates. Used in filtering.
            Vector3 jitter(const Vector3& p, int depth) const;


            // Returns the box centered at p of the same size as the leaf
            // it is in.
            AxisAlignedBox boxAt(const Vector3& p, int depth) const;


            // This function accumulates not in leaf of point p, but in all
            // leafs that intersect the given box.
            // The weight that is accumulated is divided based on overlap
            // proportion.
            void accumulateBox(const AxisAlignedBox& box, 
                const Vector3& direction, real weight, real contribution, 
                DirectionalFilter dFilter, int depth);

        };

        // How many samples per leaf node we tolerate
        int m_threshold;

        // The root of the tree
        NodePtr m_root;

    public:

        // Initializes a tree with just a root and 0 flux
        KdTree(int threshold);

        ~KdTree();

        void setThreshold(int threshold);

        // Accumulates / adds flux into the appropriate leaf node,
        // given a position and direction, during training.
        void accumulate(const Vector3& p, const Vector3& d, real weight, 
            real contribution, SpatialFilter filter, DirectionalFilter dFilter);

        // Accumulates / adds samples and flux into the appropriate leaf node,
        // given a direction, during training.
        // Assumes p is normalized to be in a hypercube (0, 0, 0) to (1, 1, 1).
        const QuadTree* getDTree(const Vector3& p) const;


        void recomputeSampleWeight();

        // Adaptively subdivides the tree's nodes if they have too much
        // flux, or prune children if they have too little. Used after
        // training, sampling, and pdf, when preparing to copy the tree
        // for the next iteration.
        // During this process, if a node is split, the children each
        // receive a quarter of its flux (information to be used while
        // subdividing, since a node's newly formed children may be
        // subdivided as well).
        void adaptTree();

        // Finally, before copying the tree to be used in the next iteration,
        // the current tree has the flux reset to 0. The new tree starts
        // anew with no accumulated flux from previous iterations, but keeps
        // the newly updated structure, and uses the previous tree
        // (after refinement, before reset), in order to sample its paths 
        // and guide them.
        void reset();

        // Returns a tree with the same structure and sample count and
        // dTrees, but all copied.
        KdTree* copyTree();

        real getSampleWeight(const Vector3& p) const;


    };

}

#endif