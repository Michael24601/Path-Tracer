#ifndef PATH_TRACER_TEXTURE_HPP
#define PATH_TRACER_TEXTURE_HPP

#include <cassert>
#include <vector>
#include "../math/vector3.hpp"

namespace pathtracer{

    class Vector2;

    class Texture{

    public:

        enum class BorderMode {
            CLAMP,
            REPEAT,
            MIRROR
        };

        enum class FilterMode {
            NEAREST,
            BILINEAR
        };

    protected:

        std::vector<std::vector<Vector3>> m_data;
        int width, height;
        BorderMode m_borderMode;
        FilterMode m_filterMode;

        // Takes in any uv coordinate, and maps it such that
        // the 0 is at the center of the first pixel, and the
        // 1 is at the center of the last pixel.
        // That means f(0) = 0.5, and f(1) = (W-0.5)
        // (where W is the maximum width or height).
        Vector2 mapToImageSpace(const Vector2& uv) const;

    public:

        Texture(const std::vector<std::vector<Vector3>>& data, 
            BorderMode borderMode, FilterMode filterMode);

        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        virtual Vector3 sample(const Vector2& uv) const;

    };

}

#endif