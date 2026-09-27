
#ifndef PATH_TRACER_QUAD_TREE_HPP
#define PATH_TRACER_QUAD_TREE_HPP

#include <atomic>
#include "config.hpp"
#include <vector>

namespace pathtracer{

    class Vector3;
    class Vector2;

    // Quadtree used to represent the flux received from each direction
    // Used in path guiding.
    class QuadTree{

    private:

        class Node;
        typedef Node* NodePtr;

        class Node{

        private:

            // The children of this node
            std::vector<NodePtr> children;

            // The flux of the node. It is atomic as multiple threads
            // may write (sum) into it. Other datafields are only
            // read by multiple threads, but since the structure of
            // the tree is not updated during path tracing (when
            // multi-threaded), there is no need to make them atomic.
            std::atomic<real> flux;

        public:

            Node(real flux);

            // Prunes the descendants of a node
            void cleanSubtree();

            bool isLeaf() const;

            void addFlux(real value);

            real getFlux() const;

            // Takes a point p, which is in local coordinates
            // (local coordinates means each node spans from (0, 0) 
            // to (1, 1)) and checks if it is in quadrant.
            bool inQuadrant(const Vector2& p) const;

            // Transform a point in the local coordinate space of
            // this node to one of its children.
            Vector2 transformToChild(const Vector2& p, int child) const;

            // Takes point in child's local space, and transforms to
            // current node's local space.
            Vector2 transformFromChild(const Vector2& p, int child) const;

            // Subdivides the node, and gives each child a quarter
            // of its flux.
            void subdivide();

            // Adds flux to node if inside its own quadrant, and leaf.
            // Else either return or check children.
            void accumulate(const Vector2& p, real contribution);

            // Recomputes the flux of the parent using the childrens',
            // but first recomputes children's flux since they may
            // be stale too.
            void recomputeFlux();

            // Samples one of the children proportional to its flux,
            // and continues down. When a lead is hit, just samples
            // the local space uniformly and returns a 2D point in
            // local space.
            // Each node then transforms the received point to their
            // own local space.
            Vector2 sample() const;

            // Returns the pdf of sampling a specific point p
            // (given in the local space of the current node)
            real pdf(const Vector2& p) const;

            // Adapts a node: if it's a leaf with too much flux 
            // subdivides it into 4 children with a quarter of the flux. 
            // Then checks the children.
            // If an internal node has too little flux, prunes its
            // descendants.
            void adaptNode(real threshold);

            // Resets the subtree (sets flux to 0)
            void reset();

            // Creates a copy of the structure of the tree, with the same flux.
            // Sets the children of newNode.
            void copy(NodePtr newNode) const;
        };

        // The root of the tree
        NodePtr m_root;

    public:

        // Initializes a tree with just a root and 0 flux
        QuadTree();

        ~QuadTree();

        real getFlux() const;

        // Accumulates / adds flux into the appropriate leaf node,
        // given a direction, during training. 
        // Each node represents a range of angles in spherical coordinates.
        void accumulate(const Vector3& direction, real contribution);

        // During the training process, leafs have their fluxes updated.
        // But their parents' fluxes remain stale. After training, this
        // recomputeFlux function is called to update the flux of every
        // internal node.
        // We could have added flux every single time we call accumulate,
        // while going down the tree, but that can add some cost,
        // as it requires using atomic operations a lot (unlike recompute
        // which is called after path tracing).
        void recomputeFlux();

        // This samples a direction from the tree (only used AFTER training 
        // and recomputing the flux). Used during rendering or while training
        // the next SD tree iteration.
        // Sampling is done hierarchically: each node chooses to visit a
        // child proportional to its flux compared to its own.
        Vector3 sample() const;

        // Returns the pdf of having sampled a direction. Again only to be
        // used after training and recomputing the flux.
        // The pdf is in solid angles.
        real pdf(const Vector3& direction) const;

        // Adaptively subdivides the tree's nodes if they have too much
        // flux, or prune children if they have too little. Used after
        // training, sampling, and pdf, when preparing to copy the tree
        // for the next iteration.
        // During this process, if a node is split, the children each
        // receive a quarter of its flux (information to be used while
        // subdividing, since a node's newly formed children may be
        // subdivided as well).
        // Note that after training a tree, we do not use the refined
        // version in rendering, refinement is for the next ieration
        // (since refining it spreads the flux evenly)
        void adaptTree();

        // Finally, before copying the tree to be used in the next iteration,
        // the current tree gets the flux reset to 0. The new tree starts
        // anew with no accumulated flux from previous iterations, but keeps
        // the newly updated structure, and uses the previous tree
        // (before refinement, before reset), in order to sample its paths 
        // and guide them.
        void reset();

        // Returns a tree with the same structure and flux
        QuadTree* copyTree();
    };

}

#endif