

#ifndef PATH_TRACER_SMS_UTIL_HPP
#define PATH_TRACER_SMS_UTIL_HPP

#include "../core/instance.hpp"
#include "../intersection/surfacePoint.hpp"
#include "../bsdf/specularBsdf.hpp"
#include "../math/matrix2.hpp"
#include "../light/lightSample.hpp"

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
                halfVector = -(wo * eta + wi).normalized();
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
                // Not sure if eta multiplies ilo or ili
                dh_du =
                    -dpdu * (ili + ilo * eta)
                    + wi * (wi.dot(dpdu) * ili)
                    + wo * (wo.dot(dpdu) * eta * ilo);

                dh_dv =
                    -dpdv * (ili + ilo * eta)
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



        static Vector3 geometricTerm(const SurfacePoint& x0,
            const SmsSample& sample, const LightSample& x2,
            const SurfaceDifferentials d1, const SurfaceDifferentials& d2) {

            static const real kMinDistance = 1e-3;
            static const real kMinDeterminant = 1e-6;

            // The specular point
            const SurfacePoint& x1 = sample.finalPoint();

            
            // Note that wi and wo are flipped in this function, so we multiply
            // wi by eta (since wi is the one connected back to x0).


            Vector3 wi = x0.position() - x1.position();
            real ili = wi.length();
            if(ili < kMinDistance){
                return 0.0;
            }
            ili = 1.0 / ili;
            wi = wi * ili;

            Vector3 wo = x2.position() - x1.position();
            real ilo = wo.length();
            if(ilo < kMinDistance){
                return 0.0;
            }
            ilo = 1.0 / ilo;
            wo = wo * ilo;

            Matrix2 dc1_dx0, dc2_dx1, dc2_dx2;
 
            // Setup generalized half-vector
            real eta = sample.eta();

            // Even though it is stored, we recompute it without normalization
            Vector3 halfVector;
            if(sample.isReflection()){
                halfVector = (wo + wi);
            }
            else{
                // This eta is eta_out / eta_in
                halfVector = (wo + wi * eta);
            }
            Vector3 h = halfVector.normalized();
            if(!sample.isReflection()) h = h * -1.0;

            // Notice eta multiplies ili, not ilo, since convention is flipped
            real ilh = 1.0 / halfVector.length();
            ilo *= ilh;
            ili *= eta * ilh;

            // Local shading tangent frame
            real dot_dpdu_n = d1.dpdu().dot(x1.shadingNormal());
            real dot_dpdv_n = d1.dpdv().dot(x1.shadingNormal());
            Vector3 s = d1.dpdu() - x1.shadingNormal() * dot_dpdu_n;
            Vector3 t = d1.dpdv() - x1.shadingNormal() * dot_dpdv_n;

            Vector3 dh_du, dh_dv;
            // Derivative of specular constraint w.r.t. x1
            dh_du = -d1.dpdu() * (ili + ilo) + wi * (wi.dot(d1.dpdu()) * ili)
                                            + wo * (wo.dot(d1.dpdu()) * ilo);
            dh_dv = -d1.dpdv() * (ili + ilo) + wi * (wi.dot(d1.dpdv()) * ili)
                                            + wo * (wo.dot(d1.dpdv()) * ilo);
            dh_du = dh_du - h * dh_du.dot(h);
            dh_dv = dh_dv - h * dh_dv.dot(h);
            if(!sample.isReflection()){
                dh_du = dh_du * -1.0;
                dh_dv = dh_dv * -1.0;
            }

            real dot_h_n    = h.dot(x1.shadingNormal());
            real dot_h_dndu = h.dot(d1.dndu());
            real dot_h_dndv = h.dot(d1.dndv());
            Matrix2 dc1_dx1(
                dh_du.dot(s) - d1.dpdu().dot(d1.dndu()) * dot_h_n - dot_dpdu_n * dot_h_dndu,
                dh_dv.dot(s) - d1.dpdu().dot(d1.dndv()) * dot_h_n - dot_dpdu_n * dot_h_dndv,
                dh_du.dot(t) - d1.dpdv().dot(d1.dndu()) * dot_h_n - dot_dpdv_n * dot_h_dndu,
                dh_dv.dot(t) - d1.dpdv().dot(d1.dndv()) * dot_h_n - dot_dpdv_n * dot_h_dndv
            );

            // Derivative of specular constraint w.r.t. x2
            dh_du = (d2.dpdu() - wo * wo.dot(d2.dpdu())) * ilo;
            dh_dv = (d2.dpdv() - wo * wo.dot(d2.dpdv())) * ilo;
            dh_du = dh_du - h * dh_du.dot(h);
            dh_dv = dh_dv - h * dh_dv.dot(h);
            if(!sample.isReflection()){
                dh_du = dh_du * -1.0;
                dh_dv = dh_dv * -1.0;
            }
            Matrix2 dc1_dx2(
                dh_du.dot(s), dh_dv.dot(s),
                dh_du.dot(t), dh_dv.dot(t)
            );


            // Invert single 2x2 matrix
            real determinant = dc1_dx1.determinant();
            if(std::abs(determinant) < kMinDeterminant){
                return 0.0;
            }
            Matrix2 inv_dc1_dx1 = dc1_dx1.inverse();
            real dx1_dx2 = std::abs((inv_dc1_dx1 * dc1_dx2).determinant());

            // Unfortunately, these geometric terms are very unstable, so to avoid
            // severe variance we need to clamp here.
            dx1_dx2 = std::min(dx1_dx2, real(1.0));

            Vector3 d = x0.position() - x1.position();
            real invR2 = 1.0 / d.lengthSquared();
            d = d * std::sqrt(invR2);

            real dw0_dx1 = std::abs(d.dot(x1.geometryNormal())) * invR2;
            real G = dw0_dx1 * dx1_dx2;
            return G;
        }


    };

}


#endif