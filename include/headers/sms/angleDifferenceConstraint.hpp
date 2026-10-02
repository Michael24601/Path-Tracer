#ifndef PATH_TRACER_ANGLE_DIFFERENCE_CONSTRAINT_HPP
#define PATH_TRACER_ANGLE_DIFFERENCE_CONSTRAINT_HPP

#include "math/vector3.hpp"
#include "math/vector2.hpp"

namespace pathtracer{

    // Forward declarations
    class SurfacePoint;
    class SurfaceDifferentials;
    class LightSample;
    class SmsSample;
    class Vector2;
    class HalfVectorDifferenitals;
    class NewtonOutput;
    

    // This is a transformed direction (reflected/refracted),
    // along with a boolean to indicate success.
    struct TransformedDirection{
        Vector3 direction;
        bool success;
    };


    struct DirectionDifferentials{
        Vector3 dwdu, dwdv;
    };


    // This also has a success flag
    struct TransformedDirectionDifferentials {
        Vector3 dsdu, dsdv;
        bool success;
    };


    struct SphericalDifferentials {
        real dtheta_du;
        real dphi_du;
        real dtheta_dv;
        real dphi_dv;
    };


    namespace AngleDifferenceConstraint{

        
        // Returns the refraction/reflection direction given the other
        // direction w.
        TransformedDirection transformDirection(const Vector3& w, 
            const Vector3& shadingNormal, bool reflection, real eta);


        // This is the angle difference constraint.
        // The specular direction is the new direction we get from
        // reflecting/refracting wo, and wi is the current specular
        // point's direction. So this compares the difference between
        // the two (as an alternative implicit function that encodes
        // the specular constraint.)
        Vector2 specularConstraint(const Vector3& transformedWo,
            const Vector3& wi);


        // Computes the derivative of wo with respect to u and v.
        // Recall wo points towards the previous point.
        // The surface differentials at the specular point are also
        // sent.
        DirectionDifferentials computeWoDifferential(
            const SurfacePoint& causticPoint, const SurfacePoint& specularPoint,
            const SurfaceDifferentials& d);


        // Computes the derivative of wi with respect to u and v.
        // Recall wi points towards the next point.
        // The surface differentials at the specular point are also
        // sent.
        DirectionDifferentials computeWiDifferential(
            const SurfacePoint& specularPoint, const LightSample& lightPoint,
            const SurfaceDifferentials& d);


        // The transform direction function takes w as input, and outputs
        // the reflected / refracted direction.
        // If u and v move, then the normal and w change,
        // which in turn changes the reflected/refracted direction. 
        // This function returns the rate of change of the reflected/refracted
        // direction when u and v move.
        TransformedDirectionDifferentials transformedDirectionDerivatives(
            const Vector3& w, const DirectionDifferentials& dw,
            const Vector3& shadingNormal, const Vector3& dndu, 
            const Vector3& dndv, bool reflection, real eta);


        // Computes the derivatives of the spherical coordinates of direction
        // w. Here w can be wi or the transformed wo (which match when
        // newton converges).
        SphericalDifferentials computerSphericalDifferentials(const Vector3& w,
            const Vector3& dwdu, const Vector3& dwdv);


        // Helper function that works for both wi and wo.
        bool computeAngleDiffConstraint(const Vector3& w,
            const DirectionDifferentials& dw, const Vector3& otherW,
            const DirectionDifferentials& otherDw,
            const Vector3& normal, const Vector3& dn_du, const Vector3& dn_dv,
            real eta, bool reflection, Matrix2& dC_dX, Vector2& C);


        // Returns the newton update step, that is, the deltaX from
        // newton equal to nabla C inverse times C.
        NewtonOutput computeNewtonStep(const SurfacePoint& x0, 
            const LightSample& x2, const SurfacePoint& p, 
            const SurfaceDifferentials& d, bool reflection, real eta);

    }

}

#endif