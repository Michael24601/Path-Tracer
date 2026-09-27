
#ifndef PATH_TRACER_QUAD_TREE_UTIL_HPP
#define PATH_TRACER_QUAD_TREE_UTIL_HPP

#include <vector>

namespace pathtracer{

    class QuadTree;
    class Vector3;

    namespace QuadTreeUtil{

        // Returns a texture that shows the directional pdf of a quadtree
        std::vector<std::vector<Vector3>> renderQuadTree(
            QuadTree* tree, int dimension);

        std::vector<std::vector<Vector3>> renderSamples(
            QuadTree* tree, int dimension, int sampleCount);

    }

}

#endif