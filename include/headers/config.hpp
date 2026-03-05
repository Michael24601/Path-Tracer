
#ifndef PATH_TRACER_CONFIG_HPP
#define PATH_TRACER_CONFIG_HPP

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <cmath>
#include <vector>
#include <string>
#include <cassert>
#include <algorithm>
#include <queue>
#include <random>

namespace pathtracer{

    // floating-point precision
    using real = double;
    inline real (*sqrtReal)(real) = ::sqrt;

}

#endif

