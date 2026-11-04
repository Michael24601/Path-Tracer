
#ifndef PATH_TRACER_CONFIG_HPP
#define PATH_TRACER_CONFIG_HPP

#include <cmath>
#include <atomic>

namespace pathtracer{

    // floating-point precision
    using real = double;
    inline real (*sqrtReal)(real) = std::sqrt;


    // Used to avoid early exit not returning an image
    extern std::atomic<bool> g_stopRendering;

}

#endif