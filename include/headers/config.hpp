
#ifndef PATH_TRACER_CONFIG_HPP
#define PATH_TRACER_CONFIG_HPP

#include <cmath>

namespace pathtracer{

    // floating-point precision
    using real = double;
    inline real (*sqrtReal)(real) = std::sqrt;

}

#endif