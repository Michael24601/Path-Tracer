
#ifndef PATH_TRACER_CONSTANTS_HPP
#define PATH_TRACER_CONSTANTS_HPP

#include "../config.hpp"

namespace pathtracer{

    // Small epsilon for floating point comparisons
    constexpr real EPSILON = 1e-8;

    // Small epsilon for self intersection tests
    constexpr real SHADOW_EPSILON = 1e-4;

    // Infinity
    constexpr real REAL_INFINITY = std::numeric_limits<double>::infinity();

    // Math constants
    constexpr real PI = 3.1415926535897932384626433832795;
    constexpr real INV_PI = 1.0 / PI;
    constexpr real PI_SQUARED = PI * PI;
    constexpr real HALF_PI = PI * 0.5;
    constexpr real TWO_PI = PI * 2.0;

}

#endif