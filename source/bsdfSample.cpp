
#include "include/headers/bsdf/bsdfSample.hpp"

namespace pathtracer{

    BsdfSample BsdfSample::INVALID = 
        BsdfSample(Vector3(0.0), Vector3::ORIGIN, 0.0, 0.0);
        
}