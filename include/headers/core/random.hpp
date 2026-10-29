#ifndef PATH_TRACER_RANDOM_HPP
#define PATH_TRACER_RANDOM_HPP

#include <random>
#include "config.hpp"

namespace pathtracer{

    class Vector2;

    class Random {

    public:

        static real next();
        
        static Vector2 next2D();
    };

}

#endif