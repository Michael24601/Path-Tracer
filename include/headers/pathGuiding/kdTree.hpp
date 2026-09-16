
#ifndef PATH_TRACER_KD_TREE_HPP
#define PATH_TRACER_KD_TREE_HPP

#include "../math/mathUtil.hpp"
#include "../core/random.hpp"
#include "quadTree.hpp"
#include "../logger.hpp"
#include <atomic>


namespace pathtracer{


    // A three dimensional KdTree
    class KdTree {

    private:

        class Node;
        typedef Node* NodePtr;

        class Node{

        private:

            // The children of this node
            std::vector<NodePtr> children;

            // The sample count in a spatial node. 
            // It is atomic as multiple threads may write (sum) into it. 
            std::atomic<int> sampleCount;

            // Directional tree (only in leafs)
            QuadTree* dTree;

        public:

            Node(int sampleCount): sampleCount{sampleCount}, 
                children(2, nullptr), dTree{nullptr} {}


            ~Node(){
                if(dTree){
                    delete dTree;
                }
            }


            // Prunes the descendants of a node
            void cleanSubtree(){

                if(isLeaf()){
                    delete dTree;
                    dTree = nullptr;
                    return;
                }

                for(int i = 0; i < 2; i++){
                    children[i]->cleanSubtree();
                    delete children[i];
                    children[i] = nullptr;
                }

                // Since this is now a leaf, we can add a dTree to it
                // (new one).
                dTree = new QuadTree();
            }


            void setDTree(QuadTree* tree){
                dTree = tree;
            }


            inline bool isLeaf() const { return !children[0]; }


            inline void addSample(){
                sampleCount.fetch_add(1, std::memory_order_relaxed);
            }


            inline void setSampleCount(int count){
                sampleCount.store(count);
            }

            
            inline int getSampleCount() const{
                return sampleCount.load(std::memory_order_relaxed);
            }


            // Takes a point p, which is in local coordinates of the
            // current node, and checks if it is inside.
            // The local coordinates are always the (0, 0, 0) to (1, 1, 1)
            // cube, and we split the axes alternating.
            inline bool inCell(const Vector3& p) const {
                return p.x() >= 0.0 && p.x() <= 1.0 &&
                    p.y() >= 0 && p.y() <= 1.0 && 
                    p.z() >= 0 && p.z() <= 1.0;
            }


            // Transform a point in the local coordinate space of
            // this node to one of its children.
            Vector3 transformToChild(const Vector3& p, int child, int depth) const {
            
                int axis = depth % 3;

                Vector3 res = p;

                if (child == 0) {
                    // Smaller side
                    res[axis] *= 2.0;
                } else {
                    // Larger side
                    res[axis] = res[axis] * 2.0 - 1.0;
                }

                return res;
            }


            // Takes point in child's local space, and transforms to
            // current node's local space.
            Vector3 transformFromChild(const Vector3& p, int child, int depth) const {
                assert(child >= 0 && child < 2);

                int axis = depth % 3;

                Vector3 res = p;

                if (child == 0) {
                    // Larger side
                    res[axis] *= 0.5;
                } else {
                    // Smaller side
                    res[axis] = res[axis] * 0.5 + 0.5;
                }

                return res;
            }


            // Subdivides the node, and gives each child half its sample count
            // and a copy of its quadTree.
            void subdivide(){

                if(!isLeaf()){
                    return;
                }

                int halfCount = getSampleCount() / 2;

                children[0] = new Node(halfCount);
                // No need to copy the first to save time
                children[0]->dTree = dTree;

                children[1] = new Node(halfCount);
                children[1]->dTree = dTree->copyTree();

                // Removes this node's tree.
                dTree = nullptr;
            }


            // This function needs to accumulate a sample. It does that
            // by going down the tree, adding the sample to the leaf
            // it arrives at, and then calling the accumulate function
            // of the dTree.
            void accumulate(const Vector3& p, const Vector3& direction, 
                real contribution, int depth){

                if(isLeaf()){
                    addSample();
                    dTree->accumulate(direction, contribution);
                    return;
                }

                // Goes into child that actually contains the point

                Vector3 p0 = transformToChild(p, 0, depth);
                Vector3 p1 = transformToChild(p, 1, depth);

                if(children[0]->inCell(p0)){
                    children[0]->accumulate(
                        transformToChild(p, 0, depth), direction, 
                        contribution, depth+1);
                }
                else {
                    children[1]->accumulate(
                        transformToChild(p, 1, depth), direction, 
                        contribution, depth+1);
                }

            }


            // Recomputes the sample count of the parent using the childrens'
            // but first recomputes children's since they may be stale too. 
            void recomputeSampleCount(){

                if(isLeaf()){
                    // We also recompute the dTree flux
                    dTree->recomputeFlux();
                    return;
                }

                int total{0};
                children[0]->recomputeSampleCount();
                children[1]->recomputeSampleCount();
                total += children[0]->getSampleCount() 
                    + children[1]->getSampleCount();

                sampleCount.store(total);
            }


            // Finds dTree in elaf associated with point p
            QuadTree* getDTree(const Vector3& p, int depth) const {

                if(isLeaf()){
                    // Just samples the tree
                    return dTree;
                }

                Vector3 p0 = transformToChild(p, 0, depth);
                Vector3 p1 = transformToChild(p, 1, depth);

                if(children[0]->inCell(p0)){
                    return children[0]->getDTree(p0, depth+1);
                }
                else {
                    return children[1]->getDTree(p1, depth+1);
                }
            }


            // Adapts a node: if it's a leaf with too many samples
            // subdivides it into 2 children with half the count. 
            // If an internal node has too little samples, prunes its
            // descendants.
            void adaptNode(int threshold){

                if(isLeaf()){

                    // First we adapt the dTree (to avoid re-adapting
                    // it if leaf is split and tree is copied)
                    dTree->adaptTree();
                    if(getSampleCount() <= threshold || threshold == 0){
                        return;
                    }

                    subdivide();
                }
                // For internal nodes, prunes children
                else if(getSampleCount() <= threshold){
                    cleanSubtree();
                    return;
                }   

                children[0]->adaptNode(threshold);
                children[1]->adaptNode(threshold);
            
            }


            // Resets the subtree (sets sample count to 0)
            // And resets all dTrees in it.
            void reset(){
                
                sampleCount.store(0.0);

                if(isLeaf()){
                    // We also reset the dTree
                    dTree->reset();
                    return;
                }

                children[0]->reset();
                children[1]->reset();
            }


            // Creates a copy of the structure of the tree, with the same flux.
            // Sets the children of newNode.
            void copy(NodePtr newNode) const{

                if(isLeaf()){
                    newNode->dTree = dTree->copyTree();
                    return;
                }

                for(int i = 0; i < 2; i++){
                    newNode->children[i] = new Node(children[i]->getSampleCount());
                    children[i]->copy(newNode->children[i]);
                }
            }

        };


        // The root of the tree
        NodePtr m_root;
        
        // How many samples per leaf node we tolerate
        int m_threshold;

    public:

        
        // Initializes a tree with just a root and 0 flux
        KdTree(int threshold) : m_threshold{threshold} {
            m_root = new Node(0);
            m_root->setDTree(new QuadTree());
        }


        ~KdTree(){
            m_root->cleanSubtree();
            delete m_root;
        }


        void setThreshold(int threshold){
            m_threshold = threshold;
        }


        // Accumulates / adds flux into the appropriate leaf node,
        // given a position and direction, during training.
        void accumulate(const Vector3& p, const Vector3& d, real contribution){
            m_root->accumulate(p, d, contribution, 0);
        }


        // Accumulates / adds samples and flux into the appropriate leaf node,
        // given a direction, during training.
        // Assumes p is normalized to be in a hypercube (0, 0, 0) to (1, 1, 1).
        QuadTree* getDTree(const Vector3& p){
            return m_root->getDTree(p, 0);
        }


        // During the training process, leafs have their fluxes updated.
        // But their parents' fluxes remain stale. After training, this
        // recomputeFlux function is called to update the flux of every
        // internal node.
        void recomputeSampleCount(){
            m_root->recomputeSampleCount();
        }


        // Adaptively subdivides the tree's nodes if they have too much
        // flux, or prune children if they have too little. Used after
        // training, sampling, and pdf, when preparing to copy the tree
        // for the next iteration.
        // During this process, if a node is split, the children each
        // receive a quarter of its flux (information to be used while
        // subdividing, since a node's newly formed children may be
        // subdivided as well).
        void adaptTree(){
            m_root->adaptNode(m_threshold);
        }


        // Finally, before copying the tree to be used in the next iteration,
        // the current tree has the flux reset to 0. The new tree starts
        // anew with no accumulated flux from previous iterations, but keeps
        // the newly updated structure, and uses the previous tree
        // (after refinement, before reset), in order to sample its paths 
        // and guide them.
        void reset(){
            m_root->reset();
        }


        // Returns a tree with the same structure and sample count and
        // dTrees, but all copied.
        KdTree* copyTree(){
            KdTree* newTree = new KdTree(m_threshold);
            newTree->m_root->setSampleCount(m_root->getSampleCount());
            m_root->copy(newTree->m_root);
            return newTree;
        }

    };

}

#endif