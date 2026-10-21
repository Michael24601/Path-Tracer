
#include "sms/smsUtil.hpp"
#include <algorithm>
#include <cmath>
#include "math/mathUtil.hpp"
#include "intersection/surfacePoint.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "math/matrix2.hpp"
#include "sms/smsSample.hpp"
#include "light/light.hpp"
#include "math/constants.hpp"
#include "shapes/shape.hpp"
#include "logger.hpp"


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



        FrameDifferentials computeFrameDifferentials(const Vector3& normal, 
            const SurfaceDifferentials& d) {

            // NOTE: the normal used here is the shading normal, not the
            // offset normal.

            Vector3 dpdu = d.dpdu();
            Vector3 dndu = d.dndu();
            Vector3 dndv = d.dndv();

            // These are only relevant for curved surfaces
            Vector3 d2pdu2  = d.d2pdu2();
            Vector3 d2pdudv = d.d2pdudv();

            Vector3 s = d.s();

            Vector3 q = dpdu - normal * normal.dot(dpdu);
            real invLength = 1.0 / q.length();

            Vector3 dq_du = d2pdu2 - dndu * normal.dot(dpdu)
                - normal * dndu.dot(dpdu)
                - normal * normal.dot(d2pdu2);

            Vector3 dq_dv = d2pdudv - dndv * normal.dot(dpdu)
                - normal * dndv.dot(dpdu)
                - normal * normal.dot(d2pdudv);

            Vector3 ds_du = (dq_du - s * s.dot(dq_du)) * invLength;
            Vector3 ds_dv = (dq_dv - s * s.dot(dq_dv)) * invLength;

            Vector3 dt_du = dndu.cross(s) + normal.cross(ds_du);
            Vector3 dt_dv = dndv.cross(s) + normal.cross(ds_dv);

            return FrameDifferentials(ds_du, ds_dv, dt_du, dt_dv);
        }



        SmsLightSample lightContribution(const LightSample& sample, const Vector3& x1){

            // For area light
            if(sample.caster()->hasArea()){

                // We need the light sample contribution, with respect to the new point.
                SurfaceSample s{sample.position(), sample.triangleIndex()};
                LightSample newSample = sample.caster()->evaluateLightSample(x1, s);

                // For the light pdf, we can use either the one from newSample
                // or the original sample, since they are the same.
                real lightPdf = newSample.pdf();

                // Since the geometric term already handles solid angle
                // conversion, we need to undo it.
                lightPdf *= (newSample.cosine()
                    / (newSample.distance() * newSample.distance()));

                Vector3 lightContribution = newSample.radiance() / lightPdf;

                return SmsLightSample{newSample, lightContribution};
            }
            else if(sample.caster()->isPoint()){

                // Point lights can be evaluated, and don't need to be,
                // since they sample the same point always. So we can just
                // use sample instead.
                LightSample newSample = sample.caster()->sample(x1);

                // We also undo solid angle conversion, even though we
                // don't have a cosine or a valid pdf for point light.
                Vector3 lightContribution = newSample.radiance() 
                    * sample.distance() * sample.distance();

                return SmsLightSample{newSample, lightContribution};
            }
            else if(sample.caster()->isDirectional()){
                // For a directional light, origin is irrelevant, so
                // we can just reuse the old sample.
                Vector3 lightContribution = sample.radiance();
                return SmsLightSample{sample, lightContribution};
            }

            LOG_ERROR("Light type is now known.");
            exit(1);
        }



        real geometricTerm(const SurfacePoint& x0, const SmsSample& sample, 
            const LightSample& x2, const SurfaceDifferentials& d0,
            const SurfaceDifferentials& d1, const SurfaceDifferentials& d2) {

            static const real kMinDistance = 1e-5;
            static const real kMinDeterminant = EPSILON;

            // The specular point
            const SurfacePoint& x1 = sample.finalPoint();

            // Note that wi and wo are flipped in this function, so we multiply
            // wi by eta (since wi is the one connected back to x0).

            Vector3 wo;
            if(x2.caster() && x2.caster()->isDirectional()){
                wo = x2.wi();
            }
            else{
                wo = (x2.position() - x1.position());
            }

            real ilo = wo.length();
            if(ilo < kMinDistance){
                return 0.0;
            }
            ilo = 1.0 / ilo;
            wo = wo * ilo;

            Vector3 wi = x0.position() - x1.position();
            real ili = wi.length();
            if(ili < kMinDistance){
                return 0.0;
            }
            ili = 1.0 / ili;
            wi = wi * ili;

            Matrix2 dc1_dx0, dc2_dx1, dc2_dx2;

            if (x2.caster()->isDirectional()) {

                // Derivative of directional light constraint with respect to x1
                Vector3 dc2_du1 = (d1.dpdu() - wo * wo.dot(d1.dpdu())) * ilo,
                        dc2_dv1 = (d1.dpdv() - wo * wo.dot(d1.dpdv())) * ilo;
                dc2_dx1 = Matrix2(
                    dc2_du1.dot(d2.dpdu()), dc2_dv1.dot(d2.dpdu()),
                    dc2_du1.dot(d2.dpdv()), dc2_dv1.dot(d2.dpdv())
                );

                // Derivative of directional light constraint with respect to x2
                Vector3 dc2_du2 = (d2.dpdu() - wo * wo.dot(d2.dpdu())) * (-ilo),
                        dc2_dv2 = (d2.dpdv() - wo * wo.dot(d2.dpdv())) * (-ilo);

                dc2_dx2 = Matrix2(
                    dc2_du2.dot(d2.dpdu()), dc2_dv2.dot(d2.dpdu()),
                    dc2_du2.dot(d2.dpdv()), dc2_dv2.dot(d2.dpdv())
                );
            }

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

            if (x2.caster()->isDirectional()) {
                // Derivative of specular constraint w.r.t. v0
                dh_du = (d0.dpdu() - wi * wi.dot(d0.dpdu())) * ili;
                dh_dv = (d0.dpdv() - wi * wi.dot(d0.dpdv())) * ili;
                dh_du = dh_du - h * dh_du.dot(h);
                dh_dv = dh_dv - h * dh_dv.dot(h);

                if (!sample.isReflection()) {
                    dh_du = -dh_du;
                    dh_dv = -dh_dv;
                }

                dc1_dx0 = Matrix2(
                    dh_du.dot(s), dh_dv.dot(s),
                    dh_du.dot(t), dh_dv.dot(t)
                );
            }

            // Derivative of specular constraint with respect to x1
            dh_du = -d1.dpdu() * (ili + ilo) + wi * (wi.dot(d1.dpdu()) * ili)
                + wo * (wo.dot(d1.dpdu()) * ilo);
            dh_dv = -d1.dpdv() * (ili + ilo) + wi * (wi.dot(d1.dpdv()) * ili)
                + wo * (wo.dot(d1.dpdv()) * ilo);
            dh_du = dh_du - h * dh_du.dot(h);
            dh_dv = dh_dv - h * dh_dv.dot(h);

            if(!sample.isReflection()){
                dh_du = -dh_du;
                dh_dv = -dh_dv;
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
                dh_du = -dh_du;
                dh_dv = -dh_dv;
            }

            Matrix2 dc1_dx2(
                dh_du.dot(s), dh_dv.dot(s),
                dh_du.dot(t), dh_dv.dot(t)
            );

            if (x2.caster()->isDirectional()) {
                // Invert 2x2 block matrix system
                real determinant = dc2_dx2.determinant();

                if (abs(determinant) < 1e-6f) {
                    return 0.0;
                }

                Matrix2 Li = dc2_dx2.inverse();
                Matrix2 tmp = Li * dc2_dx1;
                Matrix2 m = dc1_dx1 - dc1_dx2 * tmp;
                determinant = m.determinant();

                if (abs(determinant) < 1e-6f) {
                    return 0.0;
                }

                Li = m.inverse();
                Matrix2 sol1 = -Li * dc1_dx0;
                Matrix2 sol0 = -tmp * sol1;

                real G = std::abs(-sol0.determinant());
                G = std::min(G, 10.0);
                // Cancels out term that appears in bsdf
                G /= std::abs(wi.dot(x0.shadingNormal()));
                G *= (eta * eta);

                return G;
            }
            else{

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

}
