
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
#include "sms/angleDifferenceConstraint.hpp"
#include "sms/smsUtil.hpp"
#include "math/constants.hpp"
#include "logger.hpp"

namespace pathtracer{


    namespace AngleDifferenceConstraint{


        // This is the S^t / S^r from the paper.
        TransformedDirection transformDirection(const Vector3& w,
            const Vector3& shadingNormal, bool reflection, real eta) {

            // This flip is necessary to ensure consistency
            Vector3 n = shadingNormal;
            if (n.dot(w) < 0) {
                n = -n;
            }

            real cosThetaI = w.dot(n);

            if (reflection) {
                return TransformedDirection{n * 2.0 * cosThetaI - w, true};
            }

            real sin2ThetaT = (1.0 / (eta * eta)) * (1.0 - cosThetaI * cosThetaI);

            // Total internal reflection (unsuccessful).
            if (sin2ThetaT >= 1.0) {
                return TransformedDirection{Vector3(0.0), false};
            }

            real cosThetaT = std::sqrt(1.0 - sin2ThetaT);

            Vector3 dir = (-w + n * cosThetaI) / eta - n * cosThetaT;
            return TransformedDirection{dir, true};
        }


        Vector2 specularConstraint(const Vector3& transformedWo,
            const Vector3& wi) {

            Vector2 specular = SphericalCoordinates::transform(transformedWo);
            Vector2 wiSpherical = SphericalCoordinates::transform(wi);

            real dtheta = wiSpherical.x() - specular.x();
            real dphi = wiSpherical.y() - specular.y();

            if (dphi < -PI) {
                dphi += 2.0 * PI;
            } else if (dphi > PI) {
                dphi -= 2.0 * PI;
            }

            return Vector2(dtheta, dphi);
        }


        DirectionDifferentials computeWoDifferential(
            const SurfacePoint& causticPoint,
            const SurfacePoint& specularPoint,
            const SurfaceDifferentials& d
        ) {

            Vector3 wo = causticPoint.position() - specularPoint.position();
            real length = wo.length();
            wo = wo / length;

            Vector3 dpdu = d.dpdu();
            Vector3 dpdv = d.dpdv();

            return DirectionDifferentials{
                -(dpdu - wo * wo.dot(dpdu)) / length,
                -(dpdv - wo * wo.dot(dpdv)) / length
            };
        }


        DirectionDifferentials computeWiDifferential(
            const SurfacePoint& specularPoint,
            const LightSample& lightPoint,
            const SurfaceDifferentials& d) {

            Vector3 wi = lightPoint.position() - specularPoint.position();
            real length = wi.length();
            wi = wi / length;

            Vector3 dpdu = d.dpdu();
            Vector3 dpdv = d.dpdv();

            return DirectionDifferentials{
                -(dpdu - wi * wi.dot(dpdu)) / length,
                -(dpdv - wi * wi.dot(dpdv)) / length
            };
        }


        TransformedDirectionDifferentials transformedDirectionDerivatives(
            const Vector3& w, const DirectionDifferentials& dw,
            const Vector3& shadingNormal, const Vector3& dndu, 
            const Vector3& dndv, bool reflection, real eta) {

            real dotWN = w.dot(shadingNormal);

            if (reflection) {

                real dotWnDu = dw.dwdu.dot(shadingNormal) + w.dot(dndu);
                real dotWnDv = dw.dwdv.dot(shadingNormal) + w.dot(dndv);

                Vector3 dSdu = (shadingNormal * dotWnDu + dndu * dotWN)
                    * 2.0 - dw.dwdu;

                Vector3 dSdv = (shadingNormal * dotWnDv + dndv * dotWN)
                    * 2.0 - dw.dwdv;

                return {dSdu, dSdv, true};
            }

            real sin2ThetaT = (1.0 / (eta * eta)) * (1.0 - dotWN * dotWN);

            if (sin2ThetaT >= 1.0) {
                return TransformedDirectionDifferentials{
                    Vector3(0.0), Vector3(0.0), false
                };
            }

            real cosThetaT = std::sqrt(1.0 - sin2ThetaT);

            real dDotWNdu = dw.dwdu.dot(shadingNormal) + w.dot(dndu);

            real dDotWNdv = dw.dwdv.dot(shadingNormal) + w.dot(dndv);

            real dCosThetaTdu = dotWN * dDotWNdu / (eta * eta * cosThetaT);

            real dCosThetaTdv = dotWN * dDotWNdv / (eta * eta * cosThetaT);

            Vector3 dSdu = (dw.dwdu - shadingNormal * dDotWNdu - dndu * dotWN)
                / (-eta) - dndu * cosThetaT - shadingNormal * dCosThetaTdu;

            Vector3 dSdv = (dw.dwdv - shadingNormal * dDotWNdv - dndv * dotWN)
                / (-eta) - dndv * cosThetaT - shadingNormal * dCosThetaTdv;

            return TransformedDirectionDifferentials{dSdu, dSdv, true};
        }


        SphericalDifferentials computerSphericalDifferentials(const Vector3& w,
            const Vector3& dwdu, const Vector3& dwdv) {

            real d_acos = -1.0 / std::sqrt(std::max(1.0 - w.z() * w.z(), 0.0));

            real dtheta_du = d_acos * dwdu.z();
            real dtheta_dv = d_acos * dwdv.z();

            real yx = w.y() / w.x();
            real d_atan = 1.0 / (1.0 + yx * yx);

            real dphi_du = d_atan * (w.x() * dwdu.y() - w.y() * dwdu.x()) /
                (w.x() * w.x());

            real dphi_dv = d_atan * (w.x() * dwdv.y() - w.y() * dwdv.x()) /
                (w.x() * w.x());

            if (w.x() == 0.0) {
                dphi_du = 0.0;
                dphi_dv = 0.0;
            }

            return SphericalDifferentials{dtheta_du, dphi_du, dtheta_dv, 
                dphi_dv};
        }



        bool computeAngleDiffConstraint(
            const Vector3& w,
            const DirectionDifferentials& dw,
            const Vector3& otherW,
            const DirectionDifferentials& otherDw,
            const Vector3& normal,
            const Vector3& dn_du, const Vector3& dn_dv,
            real eta, bool reflection,
            Matrix2& dC_dX, Vector2& C) {

            Vector3 n = normal;
            Vector3 dndu = dn_du; 
            Vector3 dndv = dn_dv;

            if(n.dot(w) < 0){
                n = -normal;
                dndu = -dn_du;
                dndv = -dn_dv;
            }

            TransformedDirectionDifferentials ds =
                transformedDirectionDerivatives(
                    w, dw, n, dndu, dndv, reflection, eta);

            if(!ds.success){
                return false;
            }

            TransformedDirection transformed =
                transformDirection(w, n, reflection, eta);

            if(!transformed.success){
                return false;
            }

            SphericalDifferentials dTransformed =
                computerSphericalDifferentials(
                    transformed.direction, ds.dsdu, ds.dsdv);

            SphericalDifferentials dOther =
                computerSphericalDifferentials(
                    otherW, otherDw.dwdu, otherDw.dwdv);

            
            C = specularConstraint(transformed.direction, otherW);

            dC_dX = Matrix2(
                dOther.dtheta_du - dTransformed.dtheta_du,
                dOther.dtheta_dv - dTransformed.dtheta_dv,
                dOther.dphi_du - dTransformed.dphi_du,
                dOther.dphi_dv - dTransformed.dphi_dv
            );

            return true;
        }


        NewtonOutput computeNewtonStep(const SurfacePoint& x0, 
            const LightSample& x2, const SurfacePoint& p, 
            const SurfaceDifferentials& d, bool reflection, real eta){


            Vector3 wo = (x0.position() - p.position()).normalized();
            Vector3 wi = (x2.position() - p.position()).normalized();

            DirectionDifferentials dwo = computeWoDifferential(x0, p, d);
            DirectionDifferentials dwi = computeWiDifferential(p, x2, d);

            // This code needs to be updated when offset is added
            Vector3 n = p.shadingNormal();
            Vector3 dndu = d.dndu();
            Vector3 dndv = d.dndv();

            Matrix2 dC_dX;
            Vector2 C(0.0);

            bool found = computeAngleDiffConstraint(wo, dwo, wi, dwi, n, 
                dndu, dndv, eta, reflection, dC_dX, C);

            // If we cant refract/reflect with wo, we try it with wi instead.
            // This improves the number of converged paths since we try out
            // both sides before giving up on the whole path.
            if(!found){
                // It is now the opposite
                eta = 1.0 / eta;

                found = computeAngleDiffConstraint(wi, dwi, wo, dwo, n, 
                    dndu, dndv, eta, reflection, dC_dX, C);
            }

            real determinant = dC_dX.determinant();

            // This also triggers if found is false in both cases
            if (!found || std::abs(determinant) < EPSILON) {
                return NewtonOutput{
                    Matrix2::IDENTITY, Vector2(0.0), Vector2(0.0), false
                };
            }
            
            Matrix2 dX_dC = dC_dX.inverse();
            Vector2 dX = dX_dC * C;
            
            return NewtonOutput{dC_dX, dX, C, true};        
        }

    }

}
