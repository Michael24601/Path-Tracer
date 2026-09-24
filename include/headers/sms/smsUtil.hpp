

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
        // Note that this is NOT normalized. But we want to differentiate
        // the normalized one. We don't normalize it because we need its
        // length later.
        static Vector3 halfVector(const Vector3& wo, 
            const SurfacePoint& p, const Vector3& wi,
            real eta, bool reflection) {

            // Assuming wi is the one pointing towards the light
            Vector3 halfVector;
            if(reflection){
                halfVector = (wo + wi);
            }
            else{
                // This eta is eta_out / eta_in
                halfVector = -(wo * eta + wi);
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
            const Vector3& halfVector, const SurfaceDifferentials& d){

            // The constraint is T(xi)^T h(x1, x1x0, x2x0), where h
            // depends on whether it is refracting or reflecting.

            // Must be normalized;
            Vector3 h = halfVector.normalized();

            // Tangents
            Vector3 t0 = d.s();
            Vector3 t1 = d.t(p.shadingNormal());

            return Vector2(t0.dot(h), t1.dot(h));
        }


        // This computes the frame differentials from the surface
        // differentials.
        // It also needs the normal, tangent, and bitangent as inputs.
        static FrameDifferentials computeFrameDifferentials(
            const Vector3& normal,
            const SurfaceDifferentials& d) {

            Vector3 dpdu = d.dpdu();
            Vector3 dndu = d.dndu();
            Vector3 dndv = d.dndv();
            Vector3 s = d.s();

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
            const Vector3& halfVector,   // The halfvector
            const SurfaceDifferentials& s,
            bool reflection, real eta) {

            Vector3 dpdu = s.dpdu();
            Vector3 dpdv = s.dpdv();

            real ilo = 1.0 / (x0 - p.position()).length();
            real ili = 1.0 / (x2 - p.position()).length();

            real ilh = 1.0 / halfVector.length();
            Vector3 h = halfVector * ilh;

            ilo *= ilh;
            ili *= ilh;

            // We have to normalize them by the length of the halfvector

            if (!reflection) {
                ilo = ilo * eta;
            }

            Vector3 dh_du = -dpdu * (ili + ilo)
                + wi * (wi.dot(dpdu) * ili)
                + wo * (wo.dot(dpdu) * ilo);

            Vector3 dh_dv = -dpdv * (ili + ilo)
                + wi * (wi.dot(dpdv) * ili)
                + wo * (wo.dot(dpdv) * ilo);


            dh_du = dh_du - h * h.dot(dh_du);
            dh_dv = dh_dv - h * h.dot(dh_dv);

            if (!reflection) {
                dh_du = -dh_du;
                dh_dv = -dh_dv;
            }
            
            return HalfVectorDifferentials(dh_du, dh_dv);

        }


        Matrix2 computeConstraintJacobian(
            const Vector3& x0,
            const Vector3& x2,
            const SurfacePoint& p,
            const Vector3& wo,
            const Vector3& wi,
            const Vector3& halfVector,
            const SurfaceDifferentials& d,
            bool reflection,
            real eta)
        {
            Vector3 s = d.s();
            Vector3 t = d.t(p.shadingNormal());
            Vector3 h = halfVector.normalized();

            FrameDifferentials f =
                computeFrameDifferentials(p.shadingNormal(), d);

            HalfVectorDifferentials hd =
                computeHalfVectorDifferentials(
                    x0, x2, p, wo, wi, halfVector, d, reflection, eta);

            return Matrix2(
                f.dsdu.dot(h) + s.dot(hd.dhdu),
                f.dsdv.dot(h) + s.dot(hd.dhdv),
                f.dtdu.dot(h) + t.dot(hd.dhdu),
                f.dtdv.dot(h) + t.dot(hd.dhdv)
            );
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

            Vector3 s = d.s();
            Vector3 t = d.t(p.shadingNormal());
            Vector3 h = halfVector.normalized();
            
            FrameDifferentials f = computeFrameDifferentials(
                p.shadingNormal(), d);
            HalfVectorDifferentials hd = 
                computeHalfVectorDifferentials(x0, x2, p, wo, wi, 
                halfVector, d, reflection, eta);

            Matrix2 dC_dX(
                f.dsdu.dot(h) + s.dot(hd.dhdu),
                f.dsdv.dot(h) + s.dot(hd.dhdv),
                f.dtdu.dot(h) + t.dot(hd.dhdu),
                f.dtdv.dot(h) + t.dot(hd.dhdv)
            );

            Vector2 C = specularConstraint(p, halfVector, d);

            Vector2 dx = dC_dX.inverse() * C;

            return dx;
        }



        static real geometricTerm(const SurfacePoint& x0,
            const SmsSample& sample, const LightSample& x2,
            const SurfaceDifferentials d1, const SurfaceDifferentials& d2) {

            static const real kMinDistance = 1e-3;
            static const real kMinDeterminant = 1e-6;

            // The specular point
            const SurfacePoint& x1 = sample.finalPoint();


            /*
            LOG_INFO("d1.dpdu=" + std::to_string(d1.dpdu().length()) +
                    " d1.dpdv=" + std::to_string(d1.dpdv().length()) +
                    " d2.dpdu=" + std::to_string(d2.dpdu().length()) +
                    " d2.dpdv=" + std::to_string(d2.dpdv().length()) +
                    " d1.dndu=" + std::to_string(d1.dndu().length()) +
                    " d1.dndv=" + std::to_string(d1.dndv().length()));
            */

            
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
            real len = sample.halfVector().length();
            real ilh = 1.0 / len;

            // Even though it is stored, we recompute it without normalization
            Vector3 h = sample.halfVector() * ilh;
            if(!sample.isReflection()) h = h * -1.0;

            ilo *= ilh;
            // Notice eta multiplies ili not ilo
            ili *= ilh * eta;

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

            /*
            LOG_INFO("dx1_dx2=" + std::to_string(dx1_dx2) +
            " dw0_dx1=" + std::to_string(dw0_dx1) +
            " determinant=" + std::to_string(determinant) +
            " invR2=" + std::to_string(invR2) +
            " G=" + std::to_string(G));
            */

            return G;
        }


    };

}


#endif