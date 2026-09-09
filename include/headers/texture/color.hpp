
#ifndef PATH_TRACER_COLOR_HPP
#define PATH_TRACER_COLOR_HPP

#include "texture.hpp"

namespace pathtracer{

    class Color: public Texture {

    public:

        Color(const Vector3& color) : 
            Texture(std::vector<std::vector<Vector3>>{{color}},
            BorderMode::CLAMP, FilterMode::NEAREST){}


        // Overriden sample to avoid wasting computation sampling a solid
        // color.
        virtual Vector3 sample(const Vector2& uv) const override{
            return m_data[0][0];
        }

    };

}

#endif