
#include "shapes/meshUtil.hpp"
#include "core/random.hpp"
#include "shapes/triangle.hpp"

namespace pathtracer{

    namespace UniformTriangle{

        // Samples a triangle from a mesh uniformly
        int sample(const std::vector<Triangle>& triangles) {
            real rand = Random::next();
            return static_cast<int>(rand * triangles.size());
        }

        // PDF of sample triangle with this index uniformly
        real pdf(const std::vector<Triangle>& triangles, int index) {
            return (1.0f / triangles.size());
        }

    }

}