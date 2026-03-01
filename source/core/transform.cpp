
#include "include/headers/core/transform.hpp"

namespace pathtracer{

    Transform Transform::IDENTITY = Transform(Matrix3::IDENTITY, 
        Vector3::ORIGIN);
        
}