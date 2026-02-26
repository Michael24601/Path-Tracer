
#include "include/headers/intersection/intersection.hpp"

namespace pathtracer{

    Intersection Intersection::NO_HIT = Intersection(REAL_INFINITY,
        Vector3::ORIGIN, Vector3::ORIGIN, Vector3::ORIGIN, nullptr);
    
}