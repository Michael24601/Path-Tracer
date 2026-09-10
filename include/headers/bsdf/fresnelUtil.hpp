/**
 * Functions for dealing with Fresnel.
 * Based on the Fresnel implementation provided by the Lightwave framework.
 */ 

#ifndef PATH_TRACER_FRESNEL_UTIL_HPP
#define PATH_TRACER_FRESNEL_UTIL_HPP

#include "../math/constants.hpp"
#include "../math/vector3.hpp"
#include "../math/vector2.hpp"
#include "bsdf.hpp"

namespace pathtracer{

    class Fresnel{

    public:

        static real schlickWeight(real cosTheta) {
            float m = std::clamp(1 - cosTheta, 0.0, 1.0);
            return (m * m) * (m * m) * m;
        }


        static real schlick(real F0, float cosTheta) {
            return F0 + (1.0 - F0) * schlickWeight(cosTheta);
        }


        static real dielectric(real cosThetaI, real eta) {
            const float invEta = 1 / eta;
            float cosThetaTSqr = 1 - (invEta * invEta) * (1 - (cosThetaI * cosThetaI));
            if (cosThetaTSqr <= 0.0f) {
                /// total internal reflection
                return 1;
            }

            cosThetaI = std::abs(cosThetaI);
            float cosThetaT = sqrtReal(cosThetaTSqr);

            float Rs = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
            float Rp = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);

            /// Average the power of both polarizations
            return 0.5f * (Rs * Rs + Rp * Rp);
        }



    };
}

#endif