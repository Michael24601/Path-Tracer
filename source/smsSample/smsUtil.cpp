
#include "sms/smsUtil.hpp"
#include <algorithm>
#include <cmath>
#include "math/mathUtil.hpp"
#include "intersection/surfacePoint.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "math/matrix2.hpp"
#include "sms/smsSample.hpp"
#include "light/lightSample.hpp"

namespace pathtracer{


    namespace SmsUtil{

        bool isReflection(const Vector3& wo, const SurfacePoint& p, 
            const Vector3& wi) {

            // Based on the angles, we decide whether it is a reflection or
            // refraction.
            return (wo.dot(p.shadingNormal()) * wi.dot(p.shadingNormal()) > 0);
        }


        Vector3 halfVector(const Vector3& wo, const SurfacePoint& p, 
            const Vector3& wi, real eta, bool reflection) {

            // Assuming wi is the one pointing towards the light
            Vector3 halfVector;
            if(reflection){
                halfVector = (wo + wi);
            }
            else{
                // This eta is eta_out / eta_in, so I think it needs
                // to multiply wi, but im not 100% sure.
                halfVector = -(wi * eta + wo);
            }

            return halfVector;
        }


        Vector2 specularConstraint(const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& d) {

            // The constraint is T(xi)^T h(x1, x1x0, x2x0), where h
            // depends on whether it is refracting or reflecting.

            // Must be normalized;
            Vector3 h = halfVector.normalized();

            // Tangents
            Vector3 t0 = d.s();
            Vector3 t1 = d.t(p.shadingNormal());

            return Vector2(t0.dot(h), t1.dot(h));
        }


        FrameDifferentials computeFrameDifferentials(const Vector3& normal, 
            const SurfaceDifferentials& d) {

            Vector3 dpdu = d.dpdu();
            Vector3 dndu = d.dndu();
            Vector3 dndv = d.dndv();

            // These are only relevant for curved surfaces
            Vector3 d2pdu2  = d.d2pdu2();
            Vector3 d2pdudv = d.d2pdudv();

            Vector3 s = d.s();

            Vector3 q = dpdu - normal * normal.dot(dpdu);
            real invLength = 1.0 / q.length();

            Vector3 dq_du =
                d2pdu2
                - dndu * normal.dot(dpdu)
                - normal * dndu.dot(dpdu)
                - normal * normal.dot(d2pdu2);

            Vector3 dq_dv =
                d2pdudv
                - dndv * normal.dot(dpdu)
                - normal * dndv.dot(dpdu)
                - normal * normal.dot(d2pdudv);

            Vector3 ds_du = (dq_du - s * s.dot(dq_du)) * invLength;
            Vector3 ds_dv = (dq_dv - s * s.dot(dq_dv)) * invLength;

            Vector3 dt_du = dndu.cross(s) + normal.cross(ds_du);
            Vector3 dt_dv = dndv.cross(s) + normal.cross(ds_dv);

            return FrameDifferentials(ds_du, ds_dv, dt_du, dt_dv);
        }


        HalfVectorDifferentials computeHalfVectorDifferentials(
            const Vector3& x0, const Vector3& x2, const SurfacePoint& p, 
            const Vector3& wo, const Vector3& wi, const Vector3& halfVector, 
            const SurfaceDifferentials& s, bool reflection, real eta) {

            Vector3 dpdu = s.dpdu();
            Vector3 dpdv = s.dpdv();

            real ilo = 1.0 / (x0 - p.position()).length();
            real ili = 1.0 / (x2 - p.position()).length();

            real ilh = 1.0 / halfVector.length();
            Vector3 h = halfVector * ilh;

            ilo *= ilh;
            ili *= ilh;

            // We have to normalize them by the length of the halfvector

            // This eta is eta_out / eta_in, so I think it needs
            // to multiply wi, but im not 100% sure.
            if (!reflection) {
                ili = ili * eta;
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


        Vector2 computeNewtonStep(const Vector3& x0, const Vector3& x2, 
            const SurfacePoint& p, const Vector3& wo, const Vector3& wi, 
            const Vector3& halfVector, const SurfaceDifferentials& d, 
            bool reflection, real eta) {
                
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

            real determinant = dC_dX.determinant();

            if (std::abs(determinant) < 1e-6f) {
                return Vector2(0.0);
            }

            Vector2 dx = dC_dX.inverse() * C;

            return dx;
        }


        real geometricTerm(const SurfacePoint& x0, const SmsSample& sample, 
            const LightSample& x2, const SurfaceDifferentials& d1, 
            const SurfaceDifferentials& d2) {

            static const real kMinDistance = 1e-5;
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
            real len = sample.halfVector().length();
            real ilh = 1.0 / len;

            // Even though it is stored, we recompute it without normalization
            Vector3 h = sample.halfVector() * ilh;
            // No need to flip h since halfVector() does that

            ilo *= ilh;
            ili *= ilh;
            if(!sample.isReflection()){
                // This needs to be the opposite of what we multiplied
                // by eta earlier, since wo wi convention flipped inside.
                ilo *= eta;
            }

            // Local shading tangent frame

            // We recompute s and t here instead of using existing
            // ones because we want them unnormalized.
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
            real inv = 1.0 / d.length();
            d = d * inv;

            real dw0_dx1 = std::abs(d.dot(x1.geometryNormal())) * inv * inv;
            real G = dw0_dx1 * dx1_dx2;

            return G;
        }

    }

}
