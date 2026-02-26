
#ifndef PATH_TRACER_TEXTURE_HPP
#define PATH_TRACER_TEXTURE_HPP

#include "vector3.hpp"
#include "vector2.hpp"

namespace pathtracer{

    class Texture{

    private:

        enum class BorderMode {
            CLAMP,
            REPEAT,
            MIRROR
        };

        enum class FilterMode {
            NEAREST,
            BILINEAR
        };

        std::vector<std::vector<real>> m_data;
        BorderMode m_borderMode;
        FilterMode m_filterMode;


        // Takes in any uv coordinate, and maps it to 

    public:

        Texture(const std::vector<std::vector<real>>& data,
            BorderMode borderMode, FilterMode filterMode) :
            m_data{data}, m_borderMode{borderMode}, 
            m_filterMode{filterMode}{}


        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        Vector3 sample(const Vector2& uv) const{

        }

    };

}

#endif