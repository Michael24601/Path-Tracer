
#ifndef PATH_TRACER_QUAD_TREE_UTIL_HPP
#define PATH_TRACER_QUAD_TREE_UTIL_HPP

#include "quadTree.hpp"
#include "../texture/texture.hpp"

namespace pathtracer{

    class QuadTreeUtil{

    public:

        // Returns a texture that shows the directional pdf of a quadtree
        static std::vector<std::vector<Vector3>> renderQuadTree(
            QuadTree* tree, int dimension){

            std::vector<std::vector<Vector3>> data(dimension, 
                std::vector<Vector3>(dimension));

            for(int i = 0; i < dimension; i++){
                for(int j = 0; j < dimension; j++){
                    // Turns the coordinate into (0, 0) to (1, 1) space.
                    real u = (j + 0.5) / dimension;
                    real v = (i + 0.5) / dimension;
                    Vector2 coord(u, v);
                    // Converts square coordinate to direction before sending
                    Vector3 direction = SquareToSphereUniform::transform(coord);
                    real pdf = tree->pdf(direction);

                    // Visualize PDF as grayscale
                    data[i][j] = Vector3(pdf);
                }
            }

            return data;
        }

    };

}


#endif