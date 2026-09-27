#ifndef PATH_TRACER_RANDOM_HPP
#define PATH_TRACER_RANDOM_HPP

#include <random>
#include "config.hpp"

namespace pathtracer{

    class Vector2;

    class Random{

    private:

        static std::mt19937 engine;
        static std::uniform_real_distribution<real> dist;

    public:

        static real next();

        static Vector2 next2D();

    };

}

#endif