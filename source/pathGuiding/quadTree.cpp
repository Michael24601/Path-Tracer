
#include "pathGuiding/quadTree.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include "math/mathUtil.hpp"
#include "core/random.hpp"
#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "logger.hpp"
#include "bvh/axisAlignedRectangle.hpp"
#include "math/constants.hpp"


namespace pathtracer{


    QuadTree::Node::Node(real flux) :
        flux{flux},
        children(4, nullptr) {
    }


    void QuadTree::Node::cleanSubtree(){

        if(isLeaf()){
            return;
        }

        for(int i = 0; i < 4; i++){
            children[i]->cleanSubtree();
            delete children[i];
            children[i] = nullptr;
        }
    }


    bool QuadTree::Node::isLeaf() const{
        return !children[0];
    }


    void QuadTree::Node::addFlux(real value){
        // Ensures no NANs are sent here (since I have no control
        // over what the path tracer sends)
        if(!std::isfinite(value)){
            LOG_WARNING("NaN/Inf contribution: " + std::to_string(value));
            return;
        }

        flux.fetch_add(value, std::memory_order_relaxed);
    }


    real QuadTree::Node::getFlux() const{
        return flux.load(std::memory_order_relaxed);
    }


    int QuadTree::Node::childIndex(const Vector2& p) const{
        // Bit operator can be used to get the correct child index
        // based on quadrant it is in.
        return (p.x() >= 0.5 ? 1 : 0) | (p.y() >= 0.5 ? 2 : 0);
    }


    // Takes a point p, which is in local coordinates
    // (local coordinates means each node spans from (0, 0) 
    // to (1, 1)) and checks if it is in quadrant.
    bool QuadTree::Node::inQuadrant(const Vector2& p) const{
        return p.x() >= 0.0 && p.x() <= 1.0 &&
            p.y() >= 0 && p.y() <= 1.0;
    }


    Vector2 QuadTree::Node::transformToChild(
        const Vector2& p,
        int child) const {

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


    Vector2 QuadTree::Node::transformFromChild(
        const Vector2& p,
        int child) const {

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


    void QuadTree::Node::subdivide(){

        if(!isLeaf()){
            return;
        }

        real newFlux = getFlux() / 4.0;

        children[0] = new Node(newFlux);
        children[1] = new Node(newFlux);
        children[2] = new Node(newFlux);
        children[3] = new Node(newFlux);
    }


    void QuadTree::Node::accumulate(const Vector2& p, real contribution){

        if(isLeaf()){
            addFlux(contribution);
            return;
        }

        // Otherwise, check children
        int i = childIndex(p);
        children[i]->accumulate(transformToChild(p, i), contribution);
    }


    void QuadTree::Node::recomputeFlux(){

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


    void QuadTree::Node::subdivideToDepth(int depth){
        if(depth <= 0) return;

        if(isLeaf()){
            subdivide();
        }

        for(int i = 0; i < 4; i++){
            children[i]->subdivideToDepth(depth - 1);
        }
    }


    Vector2 QuadTree::Node::sample() const{

        if(isLeaf()){
            // Just samples the local space uniformly
            return Random::next2D();
        }


        // This should never happen. If it does, we just sample
        // the current node uniformly.
        if(getFlux() <= 0){
            // uniform over the node
            return Random::next2D();
        }

        // Otherwise, we sample a child: we sample a 1D
        // point, then do CDF inversion using flux as pdf.
        real randomNum = Random::next();
        real target = randomNum * getFlux();
        real cumulative = 0.0;
        int selectedChild = -1;

        for(int i = 0; i < 4; i++){
            cumulative += children[i]->getFlux();

            if(target <= cumulative){
                selectedChild = i;
                break;
            }
        }

        // If all children have 0 flux, we randomly sample the node
        if(cumulative < EPSILON){
            // uniform over the node
            return Random::next2D();
        }

        Vector2 p = children[selectedChild]->sample();

        // We transform to local space of this node before returning
        return transformFromChild(p, selectedChild);
    }

    // Returns the pdf of sampling a specific point p
    // (given in the local space of the current node)
    real QuadTree::Node::pdf(const Vector2& p) const{

        if(isLeaf()){
            // The pdf of sampling a random 2D point
            // in the local space that is from (0, 0) to (1, 1).
            return 1.0;
        }

        // If no flux, fallback is to do uniform sampling on the node
        if(getFlux() <= 0) {
            return 1.0;
        }

        // If this is the case, then we uniformly sample node
        if(getFlux() < EPSILON){
            return 1.0;
        }

        real oneOverFlux = 1.0 / getFlux();
        
        int i = childIndex(p);
        real choicePdf = (children[i]->getFlux() * oneOverFlux)
            * 4 * children[i]->pdf(transformToChild(p, i));

        return choicePdf;
    }

    // Adapts a node: if it's a leaf with too much flux 
    // subdivides it into 4 children with a quarter of the flux. 
    // Then checks the children.
    // If an internal node has too little flux, prunes its
    // descendants.
    void QuadTree::Node::adaptNode(real threshold){

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
    void QuadTree::Node::reset(){

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
    void QuadTree::Node::copy(NodePtr newNode) const{

        if(isLeaf()){
            return;
        }

        for(int i = 0; i < 4; i++){
            newNode->children[i] =
                new Node(children[i]->getFlux());

            children[i]->copy(newNode->children[i]);
        }
    }

    // Initializes a tree with just a root and 0 flux
    QuadTree::QuadTree(int baseDepth) : m_root{new Node(0.0)} {
        initialize(baseDepth);
    }


    void QuadTree::initialize(int depth){
        assert(depth >= 0);
        m_root->subdivideToDepth(depth);
    }


    QuadTree::~QuadTree(){
        m_root->cleanSubtree();
        delete m_root;
    }

    real QuadTree::getFlux() const{
        return m_root->getFlux();
    }


    real QuadTree::getFlux(const Vector3& direction) const{
        Vector2 coord = SquareToSphereUniform::inverse(direction);
        return m_root->getFlux(coord);
    }


    real QuadTree::Node::getFlux(const Vector2& p) const{

        if(isLeaf()){
            return getFlux();
        }

        // Otherwise, check children
        int i = childIndex(p);
        return children[i]->getFlux(transformToChild(p, i));
        
    }



    // Accumulates / adds flux into the appropriate leaf node,
    // given a direction, during training. 
    // Each node represents a range of angles in spherical coordinates.
    void QuadTree::accumulate(const Vector3& direction, real contribution,
        DirectionalFilter filter){

        // First we map the direction to spherical coordinate
        Vector2 coord = SquareToSphereUniform::inverse(direction);

        if(filter == DirectionalFilter::NEAREST){
            // Nothing changes here
            m_root->accumulate(coord, contribution);
        }
        else if(filter == DirectionalFilter::BOX){
            AxisAlignedRectangle box = m_root->boxAt(coord);
            // We can clip the box to ensure we don't leave the (0, 1) at root
            box.setMaxCorner(box.maxCorner().min(Vector2(1.0)));
            box.setMinCorner(box.minCorner().max(Vector2(0.0)));
            m_root->accumulateBox(box, contribution);
        }
        else{
            LOG_WARNING("Unknown filtering selected, defaulting to nearest.");
            m_root->accumulate(coord, contribution);
        }
    }

    // During the training process, leafs have their fluxes updated.
    // But their parents' fluxes remain stale. After training, this
    // recomputeFlux function is called to update the flux of every
    // internal node.
    // We could have added flux every single time we call accumulate,
    // while going down the tree, but that can add some cost,
    // as it requires using atomic operations a lot (unlike recompute
    // which is called after path tracing).
    void QuadTree::recomputeFlux(){
        m_root->recomputeFlux();
    }

    // This samples a direction from the tree (only used AFTER training 
    // and recomputing the flux). Used during rendering or while training
    // the next SD tree iteration.
    // Sampling is done hierarchically: each node chooses to visit a
    // child proportional to its flux compared to its own.
    Vector3 QuadTree::sample() const{

        // Samples a random direction by sampling one of the children
        // at each step (and at the leaf samples a point
        // on the leaf).
        Vector2 p = m_root->sample();
        return SquareToSphereUniform::transform(p);
    }

    // Returns the pdf of having sampled a direction. Again only to be
    // used after training and recomputing the flux.
    // The pdf is in solid angles.
    real QuadTree::pdf(const Vector3& direction) const{

        Vector2 coord = SquareToSphereUniform::inverse(direction);

        // We multiply by 1/4 PI to get the solid angle measure
        // (since unit sphere has 4 PI surface area, and it is
        // transformed to unit sphere).
        real p = m_root->pdf(coord) * INV_FOUR_PI;
        return p;
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
    void QuadTree::adaptTree(){

        // The threshold is 1 percent of total flux
        real threshold = m_root->getFlux() * 0.01;
        m_root->adaptNode(threshold);
    }

    // Finally, before copying the tree to be used in the next iteration,
    // the current tree gets the flux reset to 0. The new tree starts
    // anew with no accumulated flux from previous iterations, but keeps
    // the newly updated structure, and uses the previous tree
    // (before refinement, before reset), in order to sample its paths 
    // and guide them.
    void QuadTree::reset(){
        m_root->reset();
    }

    // Returns a tree with the same structure and flux
    QuadTree* QuadTree::copyTree(){

        QuadTree* newTree = new QuadTree(0);

        newTree->m_root->addFlux(m_root->getFlux());

        m_root->copy(newTree->m_root);

        return newTree;
    }


    AxisAlignedRectangle QuadTree::Node::boxAt(const Vector2& p) const{

        if (isLeaf()) {
            // Here we just return the box centered at p with 0.5 to either
            // side (size in local coordinates).
            return AxisAlignedRectangle(p - Vector2(0.5), p + Vector2(0.5));
        }

        int i = childIndex(p);
        Vector2 newP = transformToChild(p, i);
        AxisAlignedRectangle box = children[i]->boxAt(newP);

        // Back to local space
        return AxisAlignedRectangle(
            transformFromChild(box.minCorner(), i),
            transformFromChild(box.maxCorner(), i)
        );
    }


    void QuadTree::Node::accumulateBox(const AxisAlignedRectangle& box, 
        real contribution) {

        Vector2 m0 = box.minCorner();
        Vector2 m1 = box.maxCorner();

        if(isLeaf()){

            // We first need to determine the proportion of overlap
            // This is the overlap size
            Vector2 overlap = (m1.min(Vector2(1.0)) 
                - m0.max(Vector2(0.0))).max(Vector2(0.0));

            real overlapArea = overlap.x() * overlap.y();
            real boxArea = (m1.x() - m0.x()) * (m1.y() - m0.y());

            if(boxArea <= 0){
                return;
            }

            real prop = overlapArea / boxArea;

            addFlux(contribution * prop);
            return;
        }

        // Goes into every child the box overlaps
        for(int c = 0; c < 4; c++){
            Vector2 childM0 = transformToChild(m0, c);
            Vector2 childM1 = transformToChild(m1, c);

            // We hit the child if every point in max is larger than the
            // lower bound, otherwise we miss it. Same for min.
            bool hit = Vector2(0.0) <= childM1 && childM0 <= Vector2(1.0);
           
            // If it overlaps this leaf
            if(hit){
                children[c]->accumulateBox(AxisAlignedRectangle(childM0, childM1), 
                    contribution);
            }
        }

    }

}