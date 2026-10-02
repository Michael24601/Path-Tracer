#ifndef PATH_TRACER_HALF_VECTOR_CONSTRAINT_HPP
#define PATH_TRACER_HALF_VECTOR_CONSTRAINT_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    // Forward declarations
    class SurfacePoint;
    class SurfaceDifferentials;
    class LightSample;
    class SmsSample;
    class Vector2;
    class HalfVectorDifferenitals;
    class NewtonOutput;

    struct HalfVectorDifferentials{
        Vector3 dhdu, dhdv;
    };


    namespace HalfVectorConstraint{


        // Checks if specular constraint is satisfied, given the mid point
        // and the wi and wo directions. It is also sent as an argument whether
        // or not to use the reflection or refraction constraint
        // (this can be checked by the function, but it is usually
        // redundant if the caller already knows the information).
        // Note that this is the first specular constraint in the paper,
        // not the second.
        // Again, wi points towards the light.
        // The bsdf is sent to avoid having to dynamically downcast it.
        Vector2 specularConstraint(const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& d);


        HalfVectorDifferentials computeHalfVectorDifferentials(
            const SurfacePoint& x0, const LightSample& x2, const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& s, 
            bool reflection, real eta);


        // Returns the newton update step, that is, the deltaX from
        // newton equal to nabla C inverse times C.
        NewtonOutput computeNewtonStep(const SurfacePoint& x0, 
            const LightSample& x2, const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& d, 
            bool reflection, real eta);

    }

}

#endif