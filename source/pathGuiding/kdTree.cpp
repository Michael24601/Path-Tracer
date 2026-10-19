
#include "pathGuiding/kdTree.hpp"
#include <cassert>
#include "math/vector3.hpp"
#include "pathGuiding/quadTree.hpp"
#include "bvh/axisAlignedBox.hpp"
#include "core/random.hpp"
#include "logger.hpp"

namespace pathtracer{
    

    KdTree::Node::Node(real weight) :
        sampleWeight{weight},
        children(2, nullptr),
        dTree{nullptr} {
    }


    KdTree::Node::~Node(){
        if(dTree){
            delete dTree;
        }
    }


    bool KdTree::Node::inCell(const Vector3& p) const {
        return p.x() >= 0.0 && p.x() <= 1.0 &&
            p.y() >= 0 && p.y() <= 1.0 && 
            p.z() >= 0 && p.z() <= 1.0;
    }


    int KdTree::Node::childIndex(const Vector3& p, int depth) const{
        return p[depth % 3] >= 0.5 ? 1 : 0;
    }


    // Prunes the descendants of a node
    void KdTree::Node::cleanSubtree(){

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
        dTree = new QuadTree(3);
    }

    
    void KdTree::Node::setDTree(QuadTree* tree){
        dTree = tree;
    }


    // Transform a point in the local coordinate space of
    // this node to one of its children.
    Vector3 KdTree::Node::transformToChild(const Vector3& p, int child, 
        int depth) const {

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
    Vector3 KdTree::Node::transformFromChild(const Vector3& p, int child, 
        int depth) const {

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
    void KdTree::Node::subdivide(){

        if(!isLeaf()){
            return;
        }

        real halfCount = getSampleWeight() / 2.0;

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
    void KdTree::Node::accumulate(const Vector3& p, const Vector3& direction, 
        real weight, real contribution, DirectionalFilter dFilter, int depth){

        if(isLeaf()){
            addSample(weight);
            dTree->accumulate(direction, contribution, dFilter);
            return;
        }

        // Goes into child that actually contains the point
        int child = childIndex(p, depth);
        Vector3 newP = transformToChild(p, child, depth);

        children[child]->accumulate(newP, direction, weight, contribution, 
            dFilter, depth + 1);
    }


    // Recomputes the sample count of the parent using the childrens'
    // but first recomputes children's since they may be stale too. 
    void KdTree::Node::recomputeSampleWeight(){

        if(isLeaf()){
            // We also recompute the dTree flux
            dTree->recomputeFlux();
            return;
        }

        real total{0.0};

        children[0]->recomputeSampleWeight();
        children[1]->recomputeSampleWeight();

        total += children[0]->getSampleWeight()
            + children[1]->getSampleWeight();

        sampleWeight.store(total);
    }


    // Finds dTree in elaf associated with point p
    QuadTree* KdTree::Node::getDTree(const Vector3& p, int depth) const {

        if(isLeaf()){
            // Just samples the tree
            return dTree;
        }

        int child = childIndex(p, depth);
        Vector3 newP = transformToChild(p, child, depth);
        return children[child]->getDTree(newP, depth + 1);
    }


    // Adapts a node: if it's a leaf with too many samples
    // subdivides it into 2 children with half the count. 
    // If an internal node has too little samples, prunes its
    // descendants.
    void KdTree::Node::adaptNode(real threshold){

        if(isLeaf()){
            
            if(getSampleWeight() <= threshold || threshold == 0){
                // First we adapt the dTree (to avoid re-adapting
                // it if leaf is split and tree is copied)
                dTree->adaptTree();
                return;
            }

            subdivide();
        }
        
        // Kd tree does not prune children if below the threshold,
        // unlike quadtree.
        
        // If the internal node here is below the threshold, we know
        // children won't be, so no need to check them. But we recurse
        // anyway since we want to call dTree->adaptTree() on every leaf.
        children[0]->adaptNode(threshold);
        children[1]->adaptNode(threshold);
    }


    // Resets the subtree (sets sample count to 0)
    // And resets all dTrees in it.
    void KdTree::Node::reset(){

        sampleWeight.store(0.0);

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
    void KdTree::Node::copy(NodePtr newNode) const{

        if(isLeaf()){
            newNode->dTree = dTree->copyTree();
            return;
        }

        for(int i = 0; i < 2; i++){
            newNode->children[i] = new Node(children[i]->getSampleWeight());
            children[i]->copy(newNode->children[i]);
        }
    }


    // Initializes a tree with just a root and 0 flux
    KdTree::KdTree(int threshold) : m_threshold{threshold},
        m_root{new Node(0)} {
        // Initializes it with depth 3 by default
        m_root->setDTree(new QuadTree(3));
    }


    KdTree::~KdTree(){
        m_root->cleanSubtree();
        delete m_root;
    }


    void KdTree::setThreshold(int threshold){
        m_threshold = threshold;
    }


    // Accumulates / adds flux into the appropriate leaf node,
    // given a position and direction, during training.
    void KdTree::accumulate(const Vector3& p, const Vector3& d,
        real weight, real contribution, SpatialFilter filter,
        DirectionalFilter dFilter){

        if(filter == SpatialFilter::NEAREST){
            // This does nothing new
            m_root->accumulate(p, d, weight, contribution, dFilter, 0);
        }
        else if (filter == SpatialFilter::STOCHASTIC){
            // We first jitter and add it to random neighbor
            Vector3 newP = m_root->jitter(p, 0);
            // We clamp it to ensure it does not leave the scene
            newP = newP.max(Vector3(0.0)).min(Vector3(1.0));
            m_root->accumulate(newP, d, weight, contribution, dFilter, 0);    
        }
        else if (filter == SpatialFilter::BOX){
            AxisAlignedBox box = m_root->boxAt(p, 0);
            // We can clip the box to ensure we don't leave the (0, 1) at root
            box.setMaxCorner(box.maxCorner().min(Vector3(1.0)));
            box.setMinCorner(box.minCorner().max(Vector3(0.0)));
            m_root->accumulateBox(box, d, weight, contribution, dFilter, 0);
        }
        else{       
            LOG_WARNING("Unknown filtering selected, defaulting to nearest.");
            m_root->accumulate(p, d, weight, contribution, dFilter, 0);
        }
    }


    // Accumulates / adds samples and flux into the appropriate leaf node,
    // given a direction, during training.
    // Assumes p is normalized to be in a hypercube (0, 0, 0) to (1, 1, 1).
    const QuadTree* KdTree::getDTree(const Vector3& p) const{
        const QuadTree* tree = m_root->getDTree(p, 0);
        return tree;
    }


    // During the training process, leafs have their fluxes updated.
    // But their parents' fluxes remain stale. After training, this
    // recomputeFlux function is called to update the flux of every
    // internal node.
    void KdTree::recomputeSampleWeight(){
        m_root->recomputeSampleWeight();
    }


    // Adaptively subdivides the tree's nodes if they have too much
    // flux, or prune children if they have too little. Used after
    // training, sampling, and pdf, when preparing to copy the tree
    // for the next iteration.
    // During this process, if a node is split, the children each
    // receive a quarter of its flux (information to be used while
    // subdividing, since a node's newly formed children may be
    // subdivided as well).
    void KdTree::adaptTree(){
        m_root->adaptNode(m_threshold);
    }


    // Finally, before copying the tree to be used in the next iteration,
    // the current tree has the flux reset to 0. The new tree starts
    // anew with no accumulated flux from previous iterations, but keeps
    // the newly updated structure, and uses the previous tree
    // (after refinement, before reset), in order to sample its paths 
    // and guide them.
    void KdTree::reset(){
        m_root->reset();
    }


    // Returns a tree with the same structure and sample count and
    // dTrees, but all copied.
    KdTree* KdTree::copyTree(){
        KdTree* newTree = new KdTree(m_threshold);

        // We delete default quadtree to avoid memory leak when replaced
        delete newTree->m_root->getDTree();
        newTree->m_root->setDTree(nullptr);
        
        m_root->copy(newTree->m_root);
        return newTree;
    }
    

    real KdTree::Node::getSampleWeight(const Vector3& p, int depth) const {

        if(isLeaf()){
            // Just samples the tree
            return getSampleWeight();
        }

        int child = childIndex(p, depth);
        Vector3 newP = transformToChild(p, child, depth);
        return children[child]->getSampleWeight(newP, depth + 1);
    }


    real KdTree::getSampleWeight(const Vector3& p) const{
        return m_root->getSampleWeight(p, 0);
    }


    Vector3 KdTree::Node::jitter(const Vector3& p, int depth) const {

        if (isLeaf()) {
            // In local coordinates, the jitter is just -0.5 to 0.5.
            return p + Vector3(Random::next() - 0.5,
                Random::next() - 0.5,
                Random::next() - 0.5
            );
        }

        int child = childIndex(p, depth);
        Vector3 newP = transformToChild(p, child, depth);
        Vector3 res = children[child]->jitter(newP, depth + 1);

        // Back to local space
        return transformFromChild(res, child, depth);
    }


    AxisAlignedBox KdTree::Node::boxAt(const Vector3& p, int depth) const{
        if (isLeaf()) {
            // Here we just return the box centered at p with 0.5 to either
            // side (size in local coordinates).
            return AxisAlignedBox(p - Vector3(0.5), p + Vector3(0.5));
        }

        int child = childIndex(p, depth);
        Vector3 newP = transformToChild(p, child, depth);

        AxisAlignedBox box = children[child]->boxAt(newP, depth + 1);

        // Back to local space
        return AxisAlignedBox(
            transformFromChild(box.minCorner(), child, depth),
            transformFromChild(box.maxCorner(), child, depth)
        );
    }


    void KdTree::Node::accumulateBox(const AxisAlignedBox& box, 
        const Vector3& direction, real weight, real contribution, 
        DirectionalFilter dFilter, int depth) {

        Vector3 m0 = box.minCorner();
        Vector3 m1 = box.maxCorner();

        if(isLeaf()){

            // We first need to determine the proportion of overlap
            // This is the overlap size
            Vector3 overlap = (m1.min(Vector3(1.0)) 
                - m0.max(Vector3(0.0))).max(Vector3(0.0));

            real overlapVolume = overlap.x() * overlap.y() * overlap.z();
            real boxVolume = (m1.x() - m0.x()) * (m1.y() - m0.y()) 
                * (m1.z() - m0.z());

            if(boxVolume <= 0){
                return;
            }

            real prop = overlapVolume / boxVolume;

            addSample(weight * prop);
            dTree->accumulate(direction, contribution * prop, dFilter);
            return;
        }

        // Goes into every child the box overlaps
        for(int c = 0; c < 2; c++){
            Vector3 childM0 = transformToChild(m0, c, depth);
            Vector3 childM1 = transformToChild(m1, c, depth);

            // We hit the child if every point in max is larger than the
            // lower bound, otherwise we miss it. Same for min.
            bool hit = Vector3(0.0) <= childM1 && childM0 <= Vector3(1.0);
           
            // If it overlaps this leaf
            if(hit){
                children[c]->accumulateBox(AxisAlignedBox(childM0, childM1), 
                    direction, weight, contribution, dFilter, depth + 1);
            }
        }

    }


}

