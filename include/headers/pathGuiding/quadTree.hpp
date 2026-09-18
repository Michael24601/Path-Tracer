
#ifndef PATH_TRACER_QUAD_TREE_HPP
#define PATH_TRACER_QUAD_TREE_HPP

#include "../math/mathUtil.hpp"
#include "../core/random.hpp"
#include "../logger.hpp"
#include <atomic>


namespace pathtracer{


    // Quadtree used to represent the flux received from each direction
    // Used in path guiding.
    class QuadTree {

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

            Node(real flux): flux{flux}, children(4, nullptr){}


            // Prunes the descendants of a node
            void cleanSubtree(){

                if(isLeaf()){
                    return;
                }

                for(int i = 0; i < 4; i++){
                    children[i]->cleanSubtree();
                    delete children[i];
                    children[i] = nullptr;
                }
            }


            inline bool isLeaf() const { 
                return !children[0]; 
            }


            inline void addFlux(real value){
                // Ensures no NANs are sent here (since I have no control
                // over what the path tracer sends)
                if(!std::isfinite(value)){
                    LOG_WARNING("NaN/Inf contribution: " + std::to_string(value));
                    return;
                }
                flux.fetch_add(value, std::memory_order_relaxed);
            }

            
            inline real getFlux() const{
                return flux.load(std::memory_order_relaxed);
            }


            // Takes a point p, which is in local coordinates
            // (local coordinates means each node spans from (0, 0) 
            // to (1, 1)) and checks if it is in quadrant.
            inline bool inQuadrant(const Vector2& p) const {
                return p.x() >= 0.0 && p.x() <= 1.0 &&
                    p.y() >= 0 && p.y() <= 1.0;
            }


            // Transform a point in the local coordinate space of
            // this node to one of its children.
            Vector2 transformToChild(const Vector2& p, int child) const {
                assert((child >= 0 && child < 4) && "Index is wrong");

                switch (child){
                    // Bottom left
                    case 0:
                        return Vector2(2.0 * p.x(),
                            2.0 * p.y());
                    // Bottom right
                    case 1:
                        return Vector2(2.0 * p.x() - 1.0,
                            2.0 * p.y());
                    // Top left
                    case 2:
                        return Vector2(2.0 * p.x(),
                            2.0 * p.y() - 1.0);
                    // Top right
                    case 3:
                        return Vector2(2.0 * p.x() - 1.0,
                            2.0 * p.y() - 1.0);
                    default:
                        return p;
                }
            }


            // Takes point in child's local space, and transforms to
            // current node's local space.
            Vector2 transformFromChild(const Vector2& p, int child) const {
                assert(child >= 0 && child < 4);

                switch (child) {
                    case 0:
                        return Vector2(
                            0.5 * p.x(),
                            0.5 * p.y()
                        );
                    case 1:
                        return Vector2(
                            0.5 * (p.x() + 1.0),
                            0.5 * p.y()
                        );
                    case 2:
                        return Vector2(
                            0.5 * p.x(),
                            0.5 * (p.y() + 1.0)
                        );
                    case 3:
                        return Vector2(
                            0.5 * (p.x() + 1.0),
                            0.5 * (p.y() + 1.0)
                        );
                    default:
                        return p;
                }
            }


            // Subdivides the node, and gives each child a quarter
            // of its flux.
            void subdivide(){
                
                if(!isLeaf()){
                    return;
                }

                real newFlux = getFlux() / 4.0;
                children[0] = new Node(newFlux);
                children[1] = new Node(newFlux);
                children[2] = new Node(newFlux);
                children[3] = new Node(newFlux);
            }


            // Adds flux to node if inside its own quadrant, and leaf.
            // Else either return or check children.
            void accumulate(const Vector2& p, real contribution){
                if(!inQuadrant(p)){
                    return;
                }
                else{
                    if(isLeaf()){
                        addFlux(contribution);
                        return;
                    }

                    // Otherwise, check children
                    for(int i = 0; i < 4; i++){
                        children[i]->accumulate(transformToChild(p, i), contribution);
                    }

                }
            }


            // Recomputes the flux of the parent using the childrens',
            // but first recomputes children's flux since they may
            // be stale too.
            void recomputeFlux(){

                if(isLeaf()){
                    return;
                }

                real total{0.0};
                for(int i = 0; i < 4; i++){
                    children[i]->recomputeFlux();
                    real childFlux = children[i]->getFlux();
                    total += childFlux;
                }

                flux.store(total);
            }


            // Samples one of the children proportional to its flux,
            // and continues down. When a lead is hit, just samples
            // the local space uniformly and returns a 2D point in
            // local space.
            // Each node then transforms the received point to their
            // own local space.
            Vector2 sample() const {

                if(isLeaf()){
                    // Just samples the local space uniformly
                    return Random::next2D();
                }

                // This should never happen. If it does, we just sample
                // the current node uniformly.
                if(getFlux() <= 0){
                    // uniform over the 4 children
                    int c = std::min(3, static_cast<int>(Random::next() * 4));
                    return transformFromChild(children[c]->sample(), c);
                }
                
                // Otherwise, we sample a child: we sample a 1D
                // point, then do CDF inversion using flux as pdf.
                real randomNum = Random::next();
                real target = randomNum * getFlux();
                real cumulative = 0.0;
                int selectedChild = 0;

                for (int i = 0; i < 4; i++) {
                    cumulative += children[i]->getFlux();
                    if (target <= cumulative) {
                        selectedChild = i;
                        break;
                    }
                }

                Vector2 p = children[selectedChild]->sample();
                
                // We transform to local space of this node before returning
                return transformFromChild(p, selectedChild);
            }


            // Returns the pdf of sampling a specific point p
            // (given in the local space of the current node)
            real pdf(const Vector2& p) const {

                if(!inQuadrant(p)){
                    // Not sampled, so probability is 0.0
                    return 0.0;
                }

                if(isLeaf()){
                    // The pdf of sampling a random 2D point
                    // in the local space that is from (0, 0) to (1, 1).
                    return 1.0;
                }

                // If no flux, fallback is to do uniform sampling over the
                // children.
                if(getFlux() <= 0) {
                    real choicePdf{0.0};
                    for(int i = 0; i < 4; i++){
                        // Each gets multiplied by 4 and by 0.25 (uniform),
                        // which cancel out.
                        choicePdf += children[i]->pdf(transformToChild(p, i));
                    }

                    return choicePdf;
                }
                real oneOverFlux = 1.0 / getFlux();
                real choicePdf{0.0};

                for(int i = 0; i < 4; i++){
                    // Note that child pdf is transformed to local space
                    // by multiplying by 4;
                    choicePdf += (children[i]->getFlux() * oneOverFlux)
                        * 4 * children[i]->pdf(transformToChild(p, i));
                }

                return choicePdf;
            }


            // Adapts a node: if it's a leaf with too much flux 
            // subdivides it into 4 children with a quarter of the flux. 
            // Then checks the children.
            // If an internal node has too little flux, prunes its
            // descendants.
            void adaptNode(real threshold){

                if(isLeaf()){

                    // Ensures no infinite recursion
                    if(getFlux() <= threshold || threshold < EPSILON){
                        return;
                    }

                    subdivide();
                }
                // For internal nodes, prunes children
                else if(getFlux() <= threshold){
                    cleanSubtree();
                    return;
                }   

                for(int i = 0; i < 4; i++){
                    children[i]->adaptNode(threshold);
                }
            }


            // Resets the subtree (sets flux to 0)
            void reset(){
                
                flux.store(0.0);

                if(isLeaf()){
                    return;
                }

                for(int i = 0; i < 4; i++){
                    children[i]->reset();
                }
            }


            // Creates a copy of the structure of the tree, with the same flux.
            // Sets the children of newNode.
            void copy(NodePtr newNode) const{

                if(isLeaf()){
                    return;
                }

                for(int i = 0; i < 4; i++){
                    newNode->children[i] = new Node(children[i]->getFlux());
                    children[i]->copy(newNode->children[i]);
                }
            }


        };


        // The root of the tree
        NodePtr m_root;
            

    public:

        
        // Initializes a tree with just a root and 0 flux
        QuadTree() {
            m_root = new Node(0.0);
        }


        ~QuadTree(){
            m_root->cleanSubtree();
            delete m_root;
        }


        real getFlux() const{
            return m_root->getFlux();
        }


        // Accumulates / adds flux into the appropriate leaf node,
        // given a direction, during training. 
        // Each node represents a range of angles in spherical coordinates.
        void accumulate(const Vector3& direction, real contribution){

            // First we map the direction to spherical coordinate
            Vector2 coord = SquareToSphereUniform::inverse(direction);
            m_root->accumulate(coord, contribution);
        }


        // During the training process, leafs have their fluxes updated.
        // But their parents' fluxes remain stale. After training, this
        // recomputeFlux function is called to update the flux of every
        // internal node.
        // We could have added flux every single time we call accumulate,
        // while going down the tree, but that can add some cost,
        // as it requires using atomic operations a lot (unlike recompute
        // which is called after path tracing).
        void recomputeFlux(){
            m_root->recomputeFlux();
        }


        // This samples a direction from the tree (only used AFTER training 
        // and recomputing the flux). Used during rendering or while training
        // the next SD tree iteration.
        // Sampling is done hierarchically: each node chooses to visit a
        // child proportional to its flux compared to its own.
        Vector3 sample() const{
            // Samples a random direction by sampling one of the children
            // at each step (and at the leaf samples a point
            // on the leaf).
            Vector2 p = m_root->sample();
            return SquareToSphereUniform::transform(p);
        }


        // Returns the pdf of having sampled a direction. Again only to be
        // used after training and recomputing the flux.
        // The pdf is in solid angles.
        real pdf(const Vector3& direction) const{
            Vector2 coord = SquareToSphereUniform::inverse(direction);

            // We multiply by 1/4 PI to get the solid angle measure
            // (since unit sphere has 4 PI surface area, and it is
            // transformed to unit sphere).
            return m_root->pdf(coord) * INV_FOUR_PI;
        }


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
        void adaptTree(){
            // The threshold is 1 percent of total flux
            real threshold = m_root->getFlux() * 0.01;
            m_root->adaptNode(threshold);
        }


        // Finally, before copying the tree to be used in the next iteration,
        // the current tree has the flux reset to 0. The new tree starts
        // anew with no accumulated flux from previous iterations, but keeps
        // the newly updated structure, and uses the previous tree
        // (before refinement, before reset), in order to sample its paths 
        // and guide them.
        void reset(){
            m_root->reset();
        }


        // Returns a tree with the same structure and flux
        QuadTree* copyTree(){
            QuadTree* newTree = new QuadTree();
            newTree->m_root->addFlux(m_root->getFlux());
            m_root->copy(newTree->m_root);
            return newTree;
        }

    };

}

#endif