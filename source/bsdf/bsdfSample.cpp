
#include "include/headers/bsdf/bsdfSample.hpp"

namespace pathtracer{

    BsdfSample BsdfSample::INVALID = BsdfSample(Vector3::ORIGIN, 
        Vector3::ORIGIN, 0.0, -1.0);
        
}