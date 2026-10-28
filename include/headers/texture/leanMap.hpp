
#ifndef PATH_TRACER_LEAN_MAP_HPP
#define PATH_TRACER_LEAN_MAP_HPP

#include "mipMap.hpp"

namespace pathtracer{

    class Texture;
    class Matrix2;
    class Vector2;


    namespace LeanMap{

        MipMap* constructLeanMap(const Texture* normalMap);

        Vector2 mean(const MipMap* map, const Vector2& uv, int level);
        
        Matrix2 covariance(const MipMap* map, const Vector2& uv, int level);
        
    };
}

#endif