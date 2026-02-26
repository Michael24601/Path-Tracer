
#ifndef PATH_TRACER_RANDOM_HPP
#define PATH_TRACER_RANDOM_HPP

#include "vector2.hpp"

namespace pathtracer{

    class Random{

    private:

        static inline std::mt19937 engine{std::random_device{}()};
        static inline std::uniform_real_distribution<real> dist{0.0, 1.0};

    public:

        static real next(){
            return dist(engine);
        }

        static Vector2 next2D(){
            return Vector2(next(), next());
        }

    };

}

#endif