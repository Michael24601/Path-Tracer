#include "core/random.hpp"
#include "math/vector2.hpp"

namespace pathtracer{

    namespace {
        struct PCG32 {
            uint64_t state, inc;
            PCG32() {
                std::random_device rd;
                state = ((uint64_t)rd() << 32) | rd();
                inc = (((uint64_t)rd() << 32) | rd()) | 1u;
            }
            uint32_t nextU32() {
                uint64_t old = state;
                state = old * 6364136223846793005ULL + inc;
                uint32_t xorshifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
                uint32_t rot = (uint32_t)(old >> 59u);
                return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
            }
        };
        thread_local PCG32 rng;
    }

    real Random::next() {
        return (rng.nextU32() >> 8) * (1.0f / 16777216.0f);
    }


    Vector2 Random::next2D() {
        return Vector2(next(), next());
    }

}