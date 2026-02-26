
#ifndef PATH_TRACER_RAY_SHAPE_HPP
#define PATH_TRACER_RAY_SHAPE_HPP

#include "instance.hpp"

namespace pathtracer{

    class RayShape{

    private:

        // Transforms intersections from local to world coordinates
        static void transformIntersection(std::vector<Intersection>& its){
            for(Intersection& it: its){
                
            }
        }

    public:

        // Intersects a ray with a shape instance
        static std::vector<Intersection> intersect(const Ray& ray,
            const Instance* const instance){

            // First we transform the ray to local coordinates

            // Then we intersect the shape

            // The intersections are then transformed back, including
            // the distance along t.

        }

    };

}

#endif