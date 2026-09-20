

#ifndef PATH_TRACER_SMS_UTIL_HPP
#define PATH_TRACER_SMS_UTIL_HPP

#include "../core/instance.hpp"
#include "../intersection/surfacePoint.hpp"
#include "../bsdf/specularBsdf.hpp"
#include "../math/matrix2.hpp"

namespace pathtracer{


    // The derivatives of the tangent s and bitangent t
    struct FrameDifferentials{
        Vector3 dsdu, dsdv, dtdu, dtdv;
    };


    struct HalfVectorDifferentials{
        Vector3 dhdu, dhdv;
    };


    class SmsUtil{

    public:


        // Checks if three points (along with the normal at the middle)
        // constitute a possible reflection or refraction.
        // The directions are sent as input, instead of the points,
        // since they are likely already computed at the caller.
        // Here wi points towards the light.
        static bool isReflection(const Vector3& wo, const SurfacePoint& p,
            const Vector3& wi){
            // Based on the angles, we decide whether it is a reflection or
            // refraction.
            return (wo.dot(p.shadingNormal()) * wi.dot(p.shadingNormal()) > 0);
        }


        // Returns the halfvector (depends on whether we are reflecting),
        // or refracting. 
        static Vector3 halfVector(const Vector3& wo, 
            const SurfacePoint& p, const Vector3& wi,
            real eta, bool reflection) {

            // Assuming wi is the one pointing towards the light
            Vector3 halfVector;
            if(reflection){
                halfVector = (wo + wi).normalized();
            }
            else{
                // This eta is eta_out / eta_in
                halfVector = (wo * eta + wi).normalized();
            }

            return halfVector;
        }


        // Checks if specular constraint is satisfied, given the mid point
        // and the wi and wo directions. It is also sent as an argument whether
        // or not to use the reflection or refraction constraint
        // (this can be checked by the function, but it is usually
        // redundant if the caller already knows the information).
        // Note that this is the first specular constraint in the paper,
        // not the second.
        // Again, wi points towards the light.
        // The bsdf is sent to avoid having to dynamically downcast it.
        static Vector2 specularConstraint(const SurfacePoint& p,
            const Vector3& halfVector){

            // The constraint is T(xi)^T h(x1, x1x0, x2x0), where h
            // depends on whether it is refracting or reflecting.

            // Tangents
            Vector3 t0 = p.shadingFrame().getMatrix().column(0);
            Vector3 t1 = p.shadingFrame().getMatrix().column(1);

            return Vector2(t0.dot(halfVector), t1.dot(halfVector));
        }


        // This computes the frame differentials from the surface
        // differentials.
        // It also needs the normal, tangent, and bitangent as inputs.
        static FrameDifferentials computeFrameDifferentials(
            const Vector3& normal,
            const Vector3& s,
            const Vector3& t,
            const SurfaceDifferentials& d) {

            Vector3 dpdu = d.dpdu();
            Vector3 dndu = d.dndu();
            Vector3 dndv = d.dndv();

            Vector3 ds_du =
                (-dndu * normal.dot(dpdu) -
                normal * dndu.dot(dpdu)) / 
                (dpdu - normal * normal.dot(dpdu)).length();

            Vector3 ds_dv =
                (-dndv * normal.dot(dpdu) -
                normal * dndv.dot(dpdu)) /
                (dpdu - normal * normal.dot(dpdu)).length();

            ds_du = ds_du - s * ds_du.dot(s);
            ds_dv = ds_dv - s * ds_dv.dot(s);

            Vector3 dt_du = dndu.cross(s) + normal.cross(ds_du);
            Vector3 dt_dv = dndv.cross(s) + normal.cross(ds_dv);

            return FrameDifferentials(ds_du, ds_dv, dt_du, dt_dv);
        }


        static HalfVectorDifferentials computeHalfVectorDifferentials(
            const Vector3& x0, const Vector3& x2,
            const SurfacePoint& p,
            const Vector3& wo, const Vector3& wi,
            const Vector3& h,   // The halfvector
            const SurfaceDifferentials& s,
            bool reflection, real eta) {

            Vector3 dpdu = s.dpdu();
            Vector3 dpdv = s.dpdv();

            real ilo = 1.0 / (x0 - p.position()).length();
            real ili = 1.0 / (x2 - p.position()).length();

            Vector3 dh_du;
            Vector3 dh_dv;

            if (reflection) {
                dh_du =
                    -dpdu * (ili + ilo)
                    + wi * (wi.dot(dpdu) * ili)
                    + wo * (wo.dot(dpdu) * ilo);

                dh_dv =
                    -dpdv * (ili + ilo)
                    + wi * (wi.dot(dpdv) * ili)
                    + wo * (wo.dot(dpdv) * ilo);
            }
            else {
                dh_du =
                    -dpdu * (eta * ili + ilo)
                    + wi * (wi.dot(dpdu) * ili)
                    + wo * (wo.dot(dpdu) * eta * ilo);

                dh_dv =
                    -dpdv * (eta * ili + ilo)
                    + wi * (wi.dot(dpdv) * ili)
                    + wo * (wo.dot(dpdv) * eta * ilo);
            }

            dh_du = dh_du - h * h.dot(dh_du);
            dh_dv = dh_dv - h * h.dot(dh_dv);

            return HalfVectorDifferentials(dh_du, dh_dv);

        }


        // Returns the newton update step, that is, the deltaX from
        // newton equal to nabla C inverse times C.
        static Vector2 computeNewtonStep( const Vector3& x0, 
            const Vector3& x2,
            const SurfacePoint& p,
            const Vector3& wo, const Vector3& wi,
            const Vector3& halfVector,   // The halfvector
            const SurfaceDifferentials& d,
            bool reflection, real eta){

            Vector3 s = p.shadingFrame().getMatrix().column(0);
            Vector3 t = p.shadingFrame().getMatrix().column(1);
            FrameDifferentials f = computeFrameDifferentials(
                p.shadingNormal(), s, t, d);
            HalfVectorDifferentials h = 
                computeHalfVectorDifferentials(x0, x2, p, wo, wi, 
                    halfVector, d, reflection, eta);

            Matrix2 dC_dX(f.dsdu.dot(halfVector) + s.dot(h.dhdu),
                f.dsdv.dot(halfVector) + s.dot(h.dhdv),
                f.dtdu.dot(halfVector) + t.dot(h.dhdu),
                f.dtdv.dot(halfVector) + t.dot(h.dhdv));

            Vector2 C = specularConstraint(p, halfVector);
            return dC_dX.inverse() * C;
        }


    };

}


#endif