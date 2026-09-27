
#ifndef PATH_TRACER_MESH_UTIL_HPP
#define PATH_TRACER_MESH_UTIL_HPP

#include "config.hpp"
#include <vector>

namespace pathtracer{

    class Triangle;

    namespace UniformTriangle{

        // Samples a triangle from a mesh uniformly
        int sample(const std::vector<Triangle>& triangles);

        // PDF of sample triangle with this index uniformly
        real pdf(const std::vector<Triangle>& triangles, int index);

    }

}

#endif