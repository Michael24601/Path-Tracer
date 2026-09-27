#include "core/random.hpp"
#include "math/vector2.hpp"

namespace pathtracer{

    std::mt19937 Random::engine{std::random_device{}()};
    std::uniform_real_distribution<real> Random::dist{0.0, 1.0};


    real Random::next() {
        return dist(engine);
    }


    Vector2 Random::next2D() {
        return Vector2(next(), next());
    }

}