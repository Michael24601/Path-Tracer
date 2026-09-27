#ifndef PATH_TRACER_BOTTOM_LEVEL_ACCELERATION_STRUCTURE_HPP
#define PATH_TRACER_BOTTOM_LEVEL_ACCELERATION_STRUCTURE_HPP

#include <vector>
#include "bvh/axisAlignedBox.hpp"

namespace pathtracer{

    class Triangle;
    class Ray;
    class Intersection;
    
    // This is a bottom level acceleration structure (determines
    // closest hit among triangles of a mesh, not among different
    // instances).
    // That means the ray does not need to be transformed when
    // intersecting different primitives for instance.
    class BlAccelerationStructure{

    private:

        class Node;
        typedef Node* NodePtr;


        // This means that triangle X from the mesh object is at 
        // m_shapeIndexes[i] = X. We reorder the vector such that each
        // leaf contains a continuous subsection of the array.
        std::vector<int> m_shapeIndexes;

        // The root of the tree
        NodePtr m_root;

        // This means with have N bins, or N-1 different planes
        static int BIN_SIZE;

        // Number of shapes in leaf at which we stop splitting
        static int LEAF_COUNT_THRESHOLD;

        class Node{
          
        private:

            // Because each leaf contains a continuous subsection,
            // we can just store the index of the first shape it contains
            // and the number.

            // Note that when leafs reorder the vector to fit their children's
            // needs, their subsection overall is still the same (just out
            // of order), so it still works.
            int m_firstShape;
            int m_shapeCount;

            AxisAlignedBox m_box;

            NodePtr m_left;
            NodePtr m_right;

            // True by default
            bool m_isLeaf;

        public:

            Node();

            Node(int firstShape, int shapeCount);

            // Setters
            void setLeft(NodePtr left);
            void setRight(NodePtr right);
            void setLeaf(bool isLeaf);
            void setAabb(const AxisAlignedBox& box);

            // Getters
            int firstShape() const;
            int lastShape() const;
            int shapeCount() const;
            NodePtr left() const;
            NodePtr right() const;
            bool isLeaf() const;
            const AxisAlignedBox& aabb() const;

        };


        void computeAabb(NodePtr node, const std::vector<Triangle>& shapes);


        // Finds the split that minimizes the surface area heuristic
        // for a single node.
        // Then returns the split axis (x, y, or z), and the best
        // position for said axis.
        void binning(const NodePtr node, const std::vector<Triangle>& shapes,
            int& bestSplitAxis, real& bestSplitPosition);


        // Subdivides the node's shapes into two children, then recrursively
        // subdivides them.
        void subdivide(const NodePtr node, 
            const std::vector<Triangle>& shapes);


        // Finds the closest intersection and returns it if it is
        // closer than oldT (the previous intersection).
        // We assume the ray is in the correct space.
        // OldT is passed by reference in order to keep track of it
        // on all branches.
        Intersection intersectNode(const NodePtr node, real& oldT, 
            const std::vector<Triangle>& shapes, const Ray& ray) const;


    public:

        BlAccelerationStructure(const std::vector<Triangle>& shapes);

        Intersection intersect(real oldT, const std::vector<Triangle>& shapes, 
            const Ray& ray) const;
     
        
    };

}

#endif