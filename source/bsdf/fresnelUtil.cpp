#include "bsdf/fresnelUtil.hpp"
#include "math/mathUtil.hpp"
#include <algorithm>

namespace pathtracer{

    namespace Fresnel{

        real schlickWeight(real cosTheta) {
            float m = std::clamp(1 - cosTheta, 0.0, 1.0);
            return (m * m) * (m * m) * m;
        }

        real schlick(real F0, float cosTheta) {
            return F0 + (1.0 - F0) * schlickWeight(cosTheta);
        }

        real dielectric(real cosThetaI, real eta) {
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

    }
}