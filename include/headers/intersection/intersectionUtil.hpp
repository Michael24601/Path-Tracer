
#ifndef PATH_TRACER_INTERSECTION_UTIL_HPP
#define PATH_TRACER_INTERSECTION_UTIL_HPP

#include "intersection.hpp"
#include "../core/random.hpp"

namespace pathtracer{


    // Class that guards against certain intersection pitfalls like
    // self intersections.
    class IntersectionGuards{

        public:

        static bool selfIntersection(const Intersection& its){
            return its.t() < SHADOW_EPSILON;
        }


        static bool negativeDistance(const Intersection& its){
            return its.t() < SHADOW_EPSILON;
        }

    };


    class AlphaMasking{

    public:

        // Assuming we have a valid intersection, the alpha masking
        // function probabilistically chooses to either go through
        // or intersect the surface at this point, given the alpha
        // value at the intersection and using it as a probability.
        static bool alphaMask(const Intersection& its){

            // First we extract the alpha value if available
            if(!its.instance()->hasAlphaTexture()){
                return true;
            }

            real alpha = its.instance()->alphaTexture()->sample(its.uv()).x();

            real random = Random::next();
            // If alpha is 0, it means that we will always pass through.
            // If 1, it means we never pass through.
            return random < alpha;
        }

    };




}

#endif