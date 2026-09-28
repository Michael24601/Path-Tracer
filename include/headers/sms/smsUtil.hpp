#ifndef PATH_TRACER_SMS_UTIL_HPP
#define PATH_TRACER_SMS_UTIL_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    // Forward declarations
    class SurfacePoint;
    class SurfaceDifferentials;
    class LightSample;
    class SmsSample;
    class Vector2;

    // The derivatives of the tangent s and bitangent t
    struct FrameDifferentials{
        Vector3 dsdu, dsdv, dtdu, dtdv;
    };


    struct HalfVectorDifferentials{
        Vector3 dhdu, dhdv;
    };


    namespace SmsUtil{


        // Checks if three points (along with the normal at the middle)
        // constitute a possible reflection or refraction.
        // The directions are sent as input, instead of the points,
        // since they are likely already computed at the caller.
        // Here wi points towards the light.
        bool isReflection(const Vector3& wo, const SurfacePoint& p, 
            const Vector3& wi);


        // Returns the halfvector (depends on whether we are reflecting),
        // or refracting. 
        // Note that this is NOT normalized. But we want to differentiate
        // the normalized one. We don't normalize it because we need its
        // length later.
        Vector3 halfVector(const Vector3& wo, const SurfacePoint& p, 
            const Vector3& wi, real eta, bool reflection);


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


        // This computes the frame differentials from the surface
        // differentials.
        // It also needs the normal, tangent, and bitangent as inputs.
        FrameDifferentials computeFrameDifferentials(const Vector3& normal, 
            const SurfaceDifferentials& d);


        HalfVectorDifferentials computeHalfVectorDifferentials(
            const SurfacePoint& x0, const LightSample& x2, const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& s, 
            bool reflection, real eta);


        // Returns the newton update step, that is, the deltaX from
        // newton equal to nabla C inverse times C.
        Vector2 computeNewtonStep(const SurfacePoint& x0, const LightSample& x2, 
            const SurfacePoint& p, const Vector3& halfVector, 
            const SurfaceDifferentials& d, bool reflection, real eta);


        real geometricTerm(const SurfacePoint& x0, const SmsSample& sample, 
            const LightSample& x2, const SurfaceDifferentials& d1, 
            const SurfaceDifferentials& d2);

    }

}

#endif