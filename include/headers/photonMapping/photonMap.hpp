
#ifndef PATH_TRACER_PHOTON_MAP_HPP
#define PATH_TRACER_PHOTON_MAP_HPP

#include <vector>
#include "photon.hpp"

namespace pathtracer{

    class Vector3;

    // A three dimensional KdTree that stores the photons
    // Note that the axis is not necessarily in middle of each node
    class PhotonMap{

    private:

        class Node;
        typedef Node* NodePtr;

        class Node{

        private:

            // The children of this node
            NodePtr left;
            NodePtr right;

            // Firt index and number of photons in the vector of photons
            // that this node contains. This applies to both leafs
            // and parent nodes.
            int firstIndex;
            int numberOfPhotons;

            // The coordinate at which axis is placed to split left and right
            real m_splitValue;

        public:

            Node(int firstIndex, int numberOfPhotons);

            ~Node();

            inline bool isLeaf() const {
                return !left || !right;
            }

            // Subdivides the node.
            void subdivide(std::vector<Photon>& photons, int threshold, 
                int depth);


            void getPhotons(const std::vector<Photon>& photons, 
                std::vector<Photon>& inRadiusPhotons, const Vector3& center, 
                real radiusSquared, int depth) const;

        };

        // The root of the tree
        NodePtr m_root;

        // Threshold to subdivide cell
        int m_threshold;

        std::vector<Photon> m_photons;

    public:

        // Initializes a tree with the given photons
        PhotonMap(const std::vector<Photon>& photons, int threshold);

        ~PhotonMap();

        void getPhotons(std::vector<Photon>& photons, const Vector3& center, 
            real radius) const;

    };

}

#endif