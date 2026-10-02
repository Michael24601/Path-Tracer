#ifndef PATH_TRACER_SMS_UTIL_HPP
#define PATH_TRACER_SMS_UTIL_HPP

#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "math/matrix2.hpp"

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


    // Returns the matrix, dX, and the constraint C
    struct NewtonOutput{
        Matrix2 dCdx;
        Vector2 dx;
        Vector2 C;
        bool success;
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


        // This computes the frame differentials from the surface
        // differentials.
        // It also needs the normal, tangent, and bitangent as inputs.
        FrameDifferentials computeFrameDifferentials(const Vector3& normal, 
            const SurfaceDifferentials& d);


        real geometricTerm(const SurfacePoint& x0, const SmsSample& sample, 
            const LightSample& x2, const SurfaceDifferentials& d1, 
            const SurfaceDifferentials& d2);

    }

}

#endif