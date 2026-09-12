
#ifndef PATH_TRACER_BOTTOM_LEVEL_ACCELERATION_STRUCTURE_HPP
#define PATH_TRACER_BOTTOM_LEVEL_ACCELERATION_STRUCTURE_HPP

#include "../shapes/triangle.hpp"
#include <vector>

namespace pathtracer{
    
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

            Node() : m_firstShape{-1}, m_shapeCount{0}, 
                m_left{nullptr}, m_right{nullptr}, m_isLeaf{true}{}

            
            Node(int firstShape, int shapeCount) : 
                m_firstShape{firstShape}, m_shapeCount{shapeCount}, 
                m_left{nullptr}, m_right{nullptr}, m_isLeaf{true}{}

            
            // Setters
            void setLeft(NodePtr left) { m_left = left; }
            void setRight(NodePtr right) { m_right = right; }
            void setLeaf(bool isLeaf){ m_isLeaf = isLeaf; }
            void setAabb(const AxisAlignedBox& box){ m_box = box; }

            // Getters
            int firstShape() const { return m_firstShape; }
            int lastShape() const { return m_firstShape + m_shapeCount - 1; }
            int shapeCount() const { return m_shapeCount; }
            NodePtr left() const { return m_left; }
            NodePtr right() const { return m_right; }
            bool isLeaf() const { return m_isLeaf; }
            const AxisAlignedBox& aabb() const { return m_box;}

        };


        void computeAabb(NodePtr node, const std::vector<Triangle>& shapes){

            AxisAlignedBox box;
            for(int i = node->firstShape(); i <= node->lastShape(); i++){
                box.extend(shapes[m_shapeIndexes[i]].getBoundingBox());
            }
            node->setAabb(box);
        }


        // Finds the split that minimizes the surface area heuristic
        // for a single node.
        // Then returns the split axis (x, y, or z), and the best
        // position for said axis.
        void binning(const NodePtr node, const std::vector<Triangle>& shapes, 
            int& bestSplitAxis, real& bestSplitPosition) {

            float minHeuristic = REAL_INFINITY;
            // -1 by default
            int bestAxis = -1;
            // Undefined by default
            float bestSplit = 0.0f;
            
            // For each axis 
            for(int axis = 0; axis < 3; axis++){

                // We have N-1 splitting planes, however, looping over each split,
                // and then having to loop over all primitives in them
                // can be very slow.
                // Instead, we can just have a single loop over all
                // primitives, and inside that loop we can assign each
                // primitive to a bin using the centroid. Each bin keeps track of
                // the number of primitives, and AABB that encompasses all
                // of those primitives.
                // This is the advantage of binning.
                
                // We can then do two more loop over the bins accumulating
                // the bins on the left and right of each plane by combining
                // the AABB and the number of primitives; this gives us
                // the values we need to evaluate the SAH of each split.

                // The size of each bin
                float max_coord = node->aabb().maxCorner()[axis];
                float min_coord = node->aabb().minCorner()[axis];
                float size = (max_coord - min_coord) / BIN_SIZE;

                // The bin data
                std::vector<Vector3> bin_max(BIN_SIZE, Vector3(-REAL_INFINITY));
                std::vector<Vector3> bin_min(BIN_SIZE, Vector3(REAL_INFINITY));
                std::vector<int> primitive_num(BIN_SIZE, 0);

                for (int i = node->firstShape(); i <= node->lastShape(); i++) {

                    int primitiveIndex = m_shapeIndexes[i];
                    Vector3 centroid =  shapes[primitiveIndex].getCentroid();
                    AxisAlignedBox box = shapes[primitiveIndex].getBoundingBox();

                    // We need to know which bin it belongs to,
                    // unless the aabb is degenerate.
                    if(size == 0.0) continue;
                    int bin = (centroid[axis] - min_coord) / size;
                    if (bin >= BIN_SIZE) bin = BIN_SIZE - 1;
                    if(bin < 0) bin = 0;
                    
                    // We then update the bin data
                    primitive_num[bin]++;

                    // We then check min and max for x, y, z
                    for(int dir = 0; dir < 3; dir++){
                        if(box.minCorner()[dir] < bin_min[bin][dir]){
                            bin_min[bin][dir] = box.minCorner()[dir];
                        }
                        if(box.maxCorner()[dir] > bin_max[bin][dir]){
                            bin_max[bin][dir] = box.maxCorner()[dir];
                        }
                    }
                }

                std::vector<float> left_SAH(BIN_SIZE-1, 0);
                std::vector<float> right_SAH(BIN_SIZE-1, 0);

                // Now we will loop over all bins backwards and forwards,
                // accumulating the bins on the left and right.
                int total_num = 0;
                Vector3 aabb_max(-REAL_INFINITY);
                Vector3 aabb_min(REAL_INFINITY); 

                // Left pass
                for(int i = 0; i < BIN_SIZE-1; i++){
                    total_num += primitive_num[i];
                    // We then check min and max for x, y, z
                    for(int dir = 0; dir < 3; dir++){
                        if(aabb_min[dir] > bin_min[i][dir]){
                            aabb_min[dir] = bin_min[i][dir];
                        }
                        if(aabb_max[dir] < bin_max[i][dir]){
                            aabb_max[dir] = bin_max[i][dir];
                        }
                    }
                    AxisAlignedBox temp(aabb_min, aabb_max);
                    left_SAH[i] = total_num * temp.surfaceArea();
                }

                total_num = 0;
                aabb_max = Vector3(-REAL_INFINITY);
                aabb_min = Vector3(REAL_INFINITY); 

                // right pass
                for(int i = BIN_SIZE-2; i >= 0; i--){
                    total_num += primitive_num[i];
                    // We then check min and max for x, y, z
                    for(int dir = 0; dir < 3; dir++){
                        if(aabb_min[dir] > bin_min[i+1][dir]){
                            aabb_min[dir] = bin_min[i+1][dir];
                        }
                        if(aabb_max[dir] < bin_max[i+1][dir]){
                            aabb_max[dir] = bin_max[i+1][dir];
                        }
                    }
                    AxisAlignedBox temp(aabb_min, aabb_max);
                    right_SAH[i] = total_num * temp.surfaceArea();
                }

                // We finally choose the smallest SAH split
                for(int i = 0; i < BIN_SIZE-1; i++){
                    float sah = left_SAH[i] + right_SAH[i];
                    if(sah < minHeuristic){
                        minHeuristic = sah;
                        bestAxis = axis;
                        // The split coordinate with min_offset
                        bestSplit = i * size + min_coord;
                    }
                }
            }

            // Then we just set the bestAxis and split based on which
            // had teh smallest SAH value.
            bestSplitAxis = bestAxis;
            bestSplitPosition = bestSplit;
        }


        // Subdivides the node's shapes into two children, then recrursively
        // subdivides them.
        void subdivide(const NodePtr node, 
            const std::vector<Triangle>& shapes){

            // First we check if we should even split
            if(node->shapeCount() <= LEAF_COUNT_THRESHOLD){
                return;
            }

            // We then find the perfect split
            int bestSplitAxis{-1};
            real bestSplitPosition{0};
            binning(node, shapes, bestSplitAxis, bestSplitPosition);
            
            // We ensure it is valid
            if(bestSplitAxis == -1){
                return;
            }

            // We then sort the primitive array (only the shapes
            // that belong to this current node) so that we have
            // shapes to the left of the split in the left subarray.
            int firstRightIndex = node->firstShape();
            int lastLeftIndex = node->firstShape() + node->shapeCount() - 1;

            while (firstRightIndex <= lastLeftIndex) {
                if (shapes[m_shapeIndexes[firstRightIndex]].getCentroid()
                    [bestSplitAxis] < bestSplitPosition) {
                    firstRightIndex++;
                } 
                else {
                    std::swap(m_shapeIndexes[firstRightIndex], 
                        m_shapeIndexes[lastLeftIndex--]);
                }
            }

            const int firstLeftIndex = node->firstShape();
            const int leftCount = firstRightIndex - firstLeftIndex;
            const int rightCount = node->shapeCount() - leftCount;

            // If there are too few shapes on either side, no need to'
            // subdivide.
            if (leftCount == 0 || rightCount == 0) {
                return;
            }

            NodePtr left = new Node(firstLeftIndex, leftCount);
            computeAabb(left, shapes);
            NodePtr right = new Node(firstRightIndex, rightCount);
            computeAabb(right, shapes);

            node->setLeft(left);
            node->setRight(right);
            node->setLeaf(false);

            subdivide(left, shapes);
            subdivide(right, shapes);
        }



    public:


        BlAccelerationStructure(const std::vector<Triangle>& shapes){

            // First we set a (sorted) m_shapesIndexes
            m_shapeIndexes.reserve(shapes.size());
            for(int i = 0; i < shapes.size(); i++){
                m_shapeIndexes[i] = i;
            }

            m_root = new Node(0, shapes.size());
            computeAabb(m_root, shapes);
            subdivide(m_root, shapes);
        }



     
        
    };

}

#endif