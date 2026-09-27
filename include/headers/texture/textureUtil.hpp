
#ifndef PATH_TRACER_TEXTURE_UTIL_HPP
#define PATH_TRACER_TEXTURE_UTIL_HPP

#include <vector>

namespace pathtracer{

    class Vector2;
    class Vector2i;
    class Vector3;


    namespace Border{

        // Clamps the border: basically forces every value beyond
        // 0 or max to just hold the last valid value.
        // So for example, (W+1, H+2) evaluates to (W, H), 
        // and (-1, H+5) to (0, H).
        Vector2i clamp(const Vector2i& uv, int width, int height);

        // Repeats values that go beyond 0 or max, but using a modulo 
        // operator, so the texture starts again as soon as it ends.
        Vector2i repeat(const Vector2i& uv, int width, int height);

        // Mirrors the value at the borders
        Vector2i mirror(const Vector2i& uv, int width, int height);

    }


    namespace Filter{

        // Filters the texture by choosing the closest pixel to our
        // uv coordinate, by checking the decimal value of the uv
        // coordinate in image space.
        // Assumes that uv, floor, and ceilling have already
        // 
        Vector3 nearest(const Vector2& decimal, const Vector2i& floor, 
            const Vector2i& ceiling, const std::vector<std::vector<Vector3>>& data);

        // Filters the texture by doing bilinear interpolation.
        // Assumes that border handing has been done.
        Vector3 bilinear(const Vector2& decimal, const Vector2i& floor, 
            const Vector2i& ceiling, const std::vector<std::vector<Vector3>>& data);

    }

}

#endif