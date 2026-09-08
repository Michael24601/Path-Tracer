
#ifndef PATH_TRACER_MESH_UTIL_HPP
#define PATH_TRACER_MESH_UTIL_HPP

#include "triangle.hpp"

namespace pathtracer{

  
    class UniformTriangle{

    public:

        // Samples a triangle from a mesh uniformly
        static int sample(const std::vector<Triangle>& triangles){
            real rand = Random::next();
            return static_cast<int>(rand * triangles.size());
        }


        // PDF of sample triangle with this index uniformly
        static real pdf(const std::vector<Triangle>& triangles, int index){
            return (1.0f / triangles.size());
        }

    };

}

#endif