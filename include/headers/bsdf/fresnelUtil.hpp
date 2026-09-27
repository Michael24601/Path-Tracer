/**
 * Functions for dealing with Fresnel.
 * Based on the Fresnel implementation provided by the Lightwave framework.
 */

#ifndef PATH_TRACER_FRESNEL_UTIL_HPP
#define PATH_TRACER_FRESNEL_UTIL_HPP

#include "config.hpp"

namespace pathtracer{

    namespace Fresnel{

        real schlickWeight(real cosTheta);

        real schlick(real F0, float cosTheta);

        real dielectric(real cosThetaI, real eta);

    }
}

#endif