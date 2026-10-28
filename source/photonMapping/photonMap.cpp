
#include "photonMapping/photonMap.hpp"
#include <algorithm>
#include <cassert>
#include "logger.hpp"

namespace pathtracer{


    PhotonMap::Node::Node(int firstIndex, int numberOfPhotons) :
        firstIndex{firstIndex}, numberOfPhotons{numberOfPhotons},
        left{nullptr}, right{nullptr}, m_splitValue{0.0}{}


    PhotonMap::Node::~Node(){
        if(left){
            delete left;
        }
        if(right){
            delete right;
        }
    }


    // Subdivides the node.
    void PhotonMap::Node::subdivide(std::vector<Photon>& photons, int threshold,
        int depth){

        if(!isLeaf()){
            LOG_WARNING("Not a leaf, can't subdivide.");
            return;
        }

        if(numberOfPhotons <= threshold){
            return;
        }

        // We reorder the vector
        int axis = depth % 3;
        int middleIndex = firstIndex + numberOfPhotons / 2;

        auto begin = photons.begin() + firstIndex;
        auto middle = photons.begin() + middleIndex;
        auto end = photons.begin() + firstIndex + numberOfPhotons;

        // This then sorts the sub array around axis for this node
        // (the axis is not necessarily in the middle of the node,
        // but there are (give or take) the same number of nodes in each
        // of the children).
        std::nth_element(begin, middle, end,
        [axis](const Photon& a, const Photon& b){
            return a.point.position()[axis] < b.point.position()[axis];
        });

        // We need to store the split value of the axis because the middle
        // node may be jumbled if the children  are then subdivided,
        // so the split value becomes lost.
        m_splitValue = middle->point.position()[axis];

        // Then we create the children
        int leftPhotonsCount = middleIndex - firstIndex;
        left = new Node(firstIndex, leftPhotonsCount);
        right = new Node(middleIndex, numberOfPhotons - leftPhotonsCount);

        left->subdivide(photons, threshold, depth+1);
        right->subdivide(photons, threshold, depth+1);
    }


    void PhotonMap::Node::getPhotons(const std::vector<Photon>& photons, 
        std::vector<Photon>& inRadiusPhotons, const Vector3& center, 
        real radiusSquared, int depth) const {

        // If we are at a leaf, we add all of the photons that are in
        // radius. The given radius is squared to make distance
        // computation easier.
        if(isLeaf()){
            for(int i = firstIndex; i < firstIndex + numberOfPhotons; i++){
                Vector3 difference = photons[i].point.position() - center;

                if(difference.lengthSquared() <= radiusSquared){
                    inRadiusPhotons.push_back(photons[i]);
                }
            }

            return;
        }

        // If it is not a leaf, we recurse into the subtrees that can
        // be in the radius.

        int axis = depth % 3;
        real split = m_splitValue;

        // The distance between the split axis and the center, in the
        // axis coordinate.
        real distance = center[axis] - split;

        // If the center is to the left of the axis
        if(distance <= 0){

            // Then we need to check the whole left side (we have not choice)
            left->getPhotons(photons, inRadiusPhotons, center, 
                radiusSquared, depth + 1);

            // But we only need to check the right side if the axis
            // separating the right side is not too far away based
            // on the given radius of the kernel.
            if(distance * distance <= radiusSquared){
                right->getPhotons(photons, inRadiusPhotons, center, 
                    radiusSquared, depth + 1);
            }
        }
        // The same applies for the right side
        else{
            right->getPhotons(photons, inRadiusPhotons, center, 
                radiusSquared, depth + 1);

            if(distance * distance <= radiusSquared){
                left->getPhotons(photons, inRadiusPhotons, center, 
                    radiusSquared, depth + 1);
            }
        }
    }


    // Initializes a tree with the given photons
    PhotonMap::PhotonMap(const std::vector<Photon>& photons, int threshold):
        m_threshold{threshold}, m_photons{photons}{

        if(threshold < 1){
            LOG_ERROR("Threshold of photon map must be 1 or larger.");
            throw std::runtime_error("");
        }
        
        m_root = new Node(0, m_photons.size());
        m_root->subdivide(m_photons, m_threshold, 0);
    }

    PhotonMap::~PhotonMap(){
        delete m_root;
    }


    void PhotonMap::getPhotons(std::vector<Photon>& photons, 
        const Vector3& center, real radius) const {

        m_root->getPhotons(m_photons, photons, center, radius * radius, 0);
    }

}