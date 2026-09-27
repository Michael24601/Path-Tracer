
#include "pathGuiding/quadTreeUtil.hpp"
#include <algorithm>
#include "math/mathUtil.hpp"
#include "pathGuiding/quadTree.hpp"
#include "math/vector3.hpp"
#include "math/vector2.hpp"


namespace pathtracer{

    namespace QuadTreeUtil{

        // Returns a texture that shows the directional pdf of a quadtree
        std::vector<std::vector<Vector3>> renderQuadTree(
            QuadTree* tree,
            int dimension){

            std::vector<std::vector<Vector3>> data(
                dimension,
                std::vector<Vector3>(dimension));

            for(int i = 0; i < dimension; i++){
                for(int j = 0; j < dimension; j++){
                    // Turns the coordinate into (0, 0) to (1, 1) space.
                    real u = (j + 0.5) / dimension;
                    real v = (i + 0.5) / dimension;
                    Vector2 coord(u, v);

                    // Converts square coordinate to direction before sending
                    Vector3 direction =
                        SquareToSphereUniform::transform(coord);

                    real pdf = tree->pdf(direction);

                    // Visualize PDF as grayscale
                    data[i][j] = Vector3(pdf);
                }
            }

            return data;
        }

        std::vector<std::vector<Vector3>> renderSamples(
            QuadTree* tree,
            int dimension,
            int sampleCount){

            std::vector<std::vector<Vector3>> data(
                dimension,
                std::vector<Vector3>(dimension, Vector3(0.0)));

            for(int k = 0; k < sampleCount; k++){

                Vector3 direction = tree->sample();
                Vector2 coord =
                    SquareToSphereUniform::inverse(direction);

                int x = std::min(dimension - 1,
                    static_cast<int>(coord.x() * dimension));

                int y = std::min(dimension - 1,
                    static_cast<int>(coord.y() * dimension));

                data[y][x] = data[y][x] + Vector3(1.0);
            }

            real normalization =
                static_cast<real>(dimension * dimension) /
                static_cast<real>(sampleCount);

            for(int i = 0; i < dimension; i++){
                for(int j = 0; j < dimension; j++){
                    data[i][j] = data[i][j] * normalization;
                }
            }

            return data;
        }

    }
}