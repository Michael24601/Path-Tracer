
#include "bvh/blAccelerationStructure.hpp"
#include <algorithm>
#include "shapes/triangle.hpp"
#include "core/ray.hpp"
#include "intersection/intersection.hpp"
#include "math/constants.hpp"

namespace pathtracer{
    

    int BlAccelerationStructure::BIN_SIZE = 16;
    int BlAccelerationStructure::LEAF_COUNT_THRESHOLD = 4;


    BlAccelerationStructure::Node::Node() : 
        m_firstShape{-1}, 
        m_shapeCount{0}, 
        m_left{nullptr}, 
        m_right{nullptr}, 
        m_isLeaf{true}
    {}

    BlAccelerationStructure::Node::Node(int firstShape, int shapeCount) : 
        m_firstShape{firstShape}, 
        m_shapeCount{shapeCount}, 
        m_left{nullptr}, 
        m_right{nullptr}, 
        m_isLeaf{true}
    {}

    // Setters
    void BlAccelerationStructure::Node::setLeft(NodePtr left) {
        m_left = left;
    }

    void BlAccelerationStructure::Node::setRight(NodePtr right) {
        m_right = right;
    }

    void BlAccelerationStructure::Node::setLeaf(bool isLeaf) {
        m_isLeaf = isLeaf;
    }

    void BlAccelerationStructure::Node::setAabb(const AxisAlignedBox& box) {
        m_box = box;
    }

    // Getters
    int BlAccelerationStructure::Node::firstShape() const {
        return m_firstShape;
    }

    int BlAccelerationStructure::Node::lastShape() const {
        return m_firstShape + m_shapeCount - 1;
    }

    int BlAccelerationStructure::Node::shapeCount() const {
        return m_shapeCount;
    }

    BlAccelerationStructure::NodePtr 
        BlAccelerationStructure::Node::left() const {
        return m_left;
    }

    BlAccelerationStructure::NodePtr 
        BlAccelerationStructure::Node::right() const {
        return m_right;
    }

    bool BlAccelerationStructure::Node::isLeaf() const {
        return m_isLeaf;
    }

    const AxisAlignedBox& 
        BlAccelerationStructure::Node::aabb() const {
        return m_box;
    }


    void BlAccelerationStructure::computeAabb(
        NodePtr node, const std::vector<Triangle>& shapes) {

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
    void BlAccelerationStructure::binning(
        const NodePtr node, const std::vector<Triangle>& shapes,
        int& bestSplitAxis, real& bestSplitPosition) {

        real minHeuristic = REAL_INFINITY;
        int bestAxis = -1;
        real bestSplit = 0.0;

        for (int axis = 0; axis < 3; axis++)
        {
            real max_coord = node->aabb().maxCorner()[axis];
            real min_coord = node->aabb().minCorner()[axis];
            real size = (max_coord - min_coord) / BIN_SIZE;

            if (size <= 0.0)
                continue;

            std::vector<Vector3> bin_max(
                BIN_SIZE, Vector3(-REAL_INFINITY));

            std::vector<Vector3> bin_min(
                BIN_SIZE, Vector3(REAL_INFINITY));

            std::vector<int> primitive_num(BIN_SIZE, 0);

            for (int i = node->firstShape(); i <= node->lastShape(); i++)
            {
                int primitiveIndex = m_shapeIndexes[i];

                Vector3 centroid = shapes[primitiveIndex].getCentroid();
                AxisAlignedBox box = shapes[primitiveIndex].getBoundingBox();

                int bin = (centroid[axis] - min_coord) / size;

                if (bin >= BIN_SIZE)
                    bin = BIN_SIZE - 1;

                if (bin < 0)
                    bin = 0;

                primitive_num[bin]++;

                for (int dir = 0; dir < 3; dir++)
                {
                    if (box.minCorner()[dir] < bin_min[bin][dir])
                        bin_min[bin][dir] = box.minCorner()[dir];

                    if (box.maxCorner()[dir] > bin_max[bin][dir])
                        bin_max[bin][dir] = box.maxCorner()[dir];
                }
            }

            std::vector<real> left_SAH(BIN_SIZE - 1, REAL_INFINITY);
            std::vector<real> right_SAH(BIN_SIZE - 1, REAL_INFINITY);

            std::vector<int> left_num(BIN_SIZE - 1, 0);
            std::vector<int> right_num(BIN_SIZE - 1, 0);

            int total_num = 0;

            Vector3 aabb_max(-REAL_INFINITY);
            Vector3 aabb_min(REAL_INFINITY);

            for (int i = 0; i < BIN_SIZE - 1; i++)
            {
                if (primitive_num[i] > 0)
                {
                    total_num += primitive_num[i];

                    for (int dir = 0; dir < 3; dir++)
                    {
                        if (aabb_min[dir] > bin_min[i][dir])
                            aabb_min[dir] = bin_min[i][dir];

                        if (aabb_max[dir] < bin_max[i][dir])
                            aabb_max[dir] = bin_max[i][dir];
                    }
                }

                left_num[i] = total_num;

                if (total_num > 0)
                {
                    AxisAlignedBox temp(aabb_min, aabb_max);
                    left_SAH[i] = total_num * temp.surfaceArea();
                }
            }

            total_num = 0;
            aabb_max = Vector3(-REAL_INFINITY);
            aabb_min = Vector3(REAL_INFINITY);

            for (int i = BIN_SIZE - 2; i >= 0; i--)
            {
                int bin = i + 1;

                if (primitive_num[bin] > 0)
                {
                    total_num += primitive_num[bin];

                    for (int dir = 0; dir < 3; dir++)
                    {
                        if (aabb_min[dir] > bin_min[bin][dir])
                            aabb_min[dir] = bin_min[bin][dir];

                        if (aabb_max[dir] < bin_max[bin][dir])
                            aabb_max[dir] = bin_max[bin][dir];
                    }
                }

                right_num[i] = total_num;

                if (total_num > 0)
                {
                    AxisAlignedBox temp(aabb_min, aabb_max);
                    right_SAH[i] = total_num * temp.surfaceArea();
                }
            }

            for (int i = 0; i < BIN_SIZE - 1; i++)
            {
                if (left_num[i] == 0 || right_num[i] == 0)
                    continue;

                real sah = left_SAH[i] + right_SAH[i];

                if (sah < minHeuristic)
                {
                    minHeuristic = sah;
                    bestAxis = axis;

                    // Split between bin i and bin i + 1
                    bestSplit = min_coord + (i + 1) * size;
                }
            }
        }

        bestSplitAxis = bestAxis;
        bestSplitPosition = bestSplit;
    }


    // Subdivides the node's shapes into two children, then recrursively
    // subdivides them.
    void BlAccelerationStructure::subdivide(
        const NodePtr node, const std::vector<Triangle>& shapes) {

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


    // Finds the closest intersection and returns it if it is
    // closer than oldT (the previous intersection).
    // We assume the ray is in the correct space.
    // OldT is passed by reference in order to keep track of it
    // on all branches.
    Intersection BlAccelerationStructure::intersectNode(
        const NodePtr node, real& oldT, 
        const std::vector<Triangle>& shapes, const Ray& ray) const {

        // If the node is a leaf, we just text the shapes inside it
        if (node->isLeaf()) {

            Intersection result;

            for (int i = node->firstShape(); i <= node->lastShape(); i++) {
                Intersection it = shapes[m_shapeIndexes[i]].intersect(ray, oldT);

                if(it && it.t() < oldT && it.t() > SHADOW_EPSILON){
                    oldT = it.t();

                    // We need to tell the intersection which triangle was hit
                    // for reverse queries to work.
                    it.setTriangleIndex(m_shapeIndexes[i]);

                    result = it;
                }
            }

            return result;
        }

        // Otherwise, we check which of the children is closer
        // and test them first. We then use whatever t we have
        // to exclude other parts of the tree.

        real leftT = node->left()->aabb().intersect(ray);
        real rightT = node->right()->aabb().intersect(ray);

        Intersection result;

        if (leftT < rightT) {
            if (leftT < oldT){
                // oldT is updated here
                Intersection it = intersectNode(
                    node->left(), oldT, shapes, ray);

                // Result is only valid (true) if it was closer than oldT
                if(it){
                    result = it;
                }
            }

            if (rightT < oldT){
                // oldT is updated here
                Intersection it = intersectNode(
                    node->right(), oldT, shapes, ray);

                // Result is only valid (true) if it was closer than oldT
                if(it){
                    result = it;
                }
            }
        } else {
            if (rightT < oldT){
                // oldT is updated here
                Intersection it = intersectNode(
                    node->right(), oldT, shapes, ray);

                // Result is only valid (true) if it was closer than oldT
                if(it){
                    result = it;
                }
            }

            if (leftT < oldT){
                // oldT is updated here
                Intersection it = intersectNode(
                    node->left(), oldT, shapes, ray);

                // Result is only valid (true) if it was closer than oldT
                if(it){
                    result = it;
                }
            }
        }

        return result;
    }


    BlAccelerationStructure::BlAccelerationStructure(
        const std::vector<Triangle>& shapes) {

        // First we set a (sorted) m_shapesIndexes
        m_shapeIndexes.resize(shapes.size());

        for(int i = 0; i < shapes.size(); i++){
            m_shapeIndexes[i] = i;
        }

        m_root = new Node(0, shapes.size());
        computeAabb(m_root, shapes);
        subdivide(m_root, shapes);
    }


    Intersection BlAccelerationStructure::intersect(
        real oldT, const std::vector<Triangle>& shapes, 
        const Ray& ray) const {

        real copyT = oldT;
        return intersectNode(m_root, copyT, shapes, ray);
    }


}