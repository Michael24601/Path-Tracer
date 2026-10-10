
#include "sms/smsUtil.hpp"
#include <algorithm>
#include <cmath>
#include "math/mathUtil.hpp"
#include "intersection/surfacePoint.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "math/matrix2.hpp"
#include "sms/smsSample.hpp"
#include "light/lightSample.hpp"
#include "light/light.hpp"
#include "sms/halfVectorConstraint.hpp"
#include "sms/smsUtil.hpp"

namespace pathtracer{


    namespace HalfVectorConstraint{


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


        HalfVectorDifferentials computeHalfVectorDifferentials(
            const SurfacePoint& x0, const LightSample& x2,  const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& s, 
            bool reflection, real eta) {

            Vector3 dpdu = s.dpdu();
            Vector3 dpdv = s.dpdv();

            Vector3 wo = x0.position() - p.position();

            // If directional, direction does not change (and position is not
            // finite so can't be used).
            Vector3 wi;
            if(x2.caster() && x2.caster()->isDirectional()){
                wi = x2.wi();
            }
            else{
                wi = (x2.position() - p.position());
            }

            real ilo = 1.0 / wo.length();
            wo = wo * ilo;
            real ili = 1.0 / wi.length();
            wi = wi * ili;

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


        NewtonOutput computeNewtonStep(const SurfacePoint& x0, 
            const LightSample& x2, const SurfacePoint& p, 
            const Vector3& halfVector, const SurfaceDifferentials& d, 
            bool reflection, real eta) {
                
            Vector3 s = d.s();
            Vector3 t = d.t(p.shadingNormal());
            Vector3 h = halfVector.normalized();

            FrameDifferentials f = SmsUtil::computeFrameDifferentials(
                p.shadingNormal(), d);

            HalfVectorDifferentials hd =
                computeHalfVectorDifferentials(x0, x2, p, halfVector, d, 
                    reflection, eta);

            Matrix2 dC_dX(
                f.dsdu.dot(h) + s.dot(hd.dhdu),
                f.dsdv.dot(h) + s.dot(hd.dhdv),
                f.dtdu.dot(h) + t.dot(hd.dhdu),
                f.dtdv.dot(h) + t.dot(hd.dhdv)
            );

            Vector2 C = specularConstraint(p, halfVector, d);

            real determinant = dC_dX.determinant();

            if (std::abs(determinant) < 1e-6f) {
                return NewtonOutput{
                    Matrix2::IDENTITY, Vector2(0.0), Vector2(0.0), false
                };
            }

            Vector2 dx = dC_dX.inverse() * C;

            return NewtonOutput{dC_dX, dx, C, true};
        }

    }

}
