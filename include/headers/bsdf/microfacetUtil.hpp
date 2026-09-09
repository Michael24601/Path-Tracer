/**
 * Functions for dealing with microfacet distributions.
 * Based on the microfacet distribution implementation provided by
 * the Lightwave framework.
 */ 

#ifndef PATH_TRACER_MICROFACET_UTIL_HPP
#define PATH_TRACER_MICROFACET_UTIL_HPP

#include "../math/constants.hpp"
#include "../math/vector3.hpp"
#include "../math/vector2.hpp"
#include "bsdf.hpp"

namespace pathtracer{

    class Microfacet{

    public:


        static real smithG1(real alpha, const Vector3& wh, const Vector3& w) {
            if (w.dot(wh) * Bsdf::cosineTheta(w) * Bsdf::cosineTheta(wh) <= 0){
                return 0;
            }
            if (Bsdf::absCosineTheta(w) >= 1){
                return 1;
            }

            const real cos = Bsdf::cosineTheta(w);
            const real tanTheta2 = (1 - cos * cos) / (cos * cos);
            const real a2tanTheta2 = (alpha * alpha) * tanTheta2;
            return 2 / (1 + sqrtReal(1 + a2tanTheta2));
        }


        static real evaluateGGX(real alpha, const Vector3& wh) {
            real nDotH = Bsdf::cosineTheta(wh);
            real a = Bsdf::cosinePhiSineTheta(wh) / alpha;
            real b = Bsdf::sinePhiSineTheta(wh) / alpha;
            real c = (a * a) + (b * b) + (nDotH * nDotH);
            return 1 / (PI * (alpha * c) * (alpha * c));
        }



        static Vector3 sampleGGXVNDF(real alpha, const Vector3 &wo, 
            const Vector2 &rnd) {

            real sgn = copysign(1, Bsdf::cosineTheta(wo));

            Vector3 Vh =
                Vector3(alpha * wo.x(), alpha * wo.y(), wo.z()).normalized() * sgn;

            real lensq = Vh.x() * Vh.x() + Vh.y() * Vh.y();

            Vector3 T1 =
                lensq > 0
                    ? Vector3(-Vh.y(), Vh.x(), 0) * (1.0 / sqrtReal(lensq))
                    : Vector3(1, 0, 0);

            Vector3 T2 = Vh.cross(T1);

            real r   = sqrtReal(rnd.x());
            real phi = TWO_PI * rnd.y();
            real t1  = r * cos(phi);
            real t2  = r * sin(phi);
            real s   = 0.5f * (1 + Vh.z());

            t2 = (1 - s) * sqrtReal(1 - (t1) * (t1)) + s * t2;

            Vector3 Nh =
                T1 * t1 +
                T2 * t2 +
                Vh * sqrtReal(1 - (t1) * (t1) - (t2) * (t2));

            Vector3 Ne = Vector3(alpha * Nh.x(), alpha * Nh.y(), 
                std::max(0.0, Nh.z())).normalized();

            return Ne * sgn;
        }


        static real pdfGGXVNDF(real alpha, const Vector3& wh, const Vector3& wo) {
            return evaluateGGX(alpha, wh) *
                smithG1(alpha, wh, wo) *
                abs(wh.dot(wo)) / Bsdf::absCosineTheta(wo);
        }


    };
}

#endif