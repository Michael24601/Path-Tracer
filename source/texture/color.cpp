
#include "texture/color.hpp"

namespace pathtracer{

    Color::Color(const Vector3& color) :
        // Not linearized
        Texture(std::vector<std::vector<Vector3>>{{color}},
            BorderMode::CLAMP, FilterMode::NEAREST, false) {
    }


    Vector3 Color::sample(const Vector2& uv) const {
        return m_data[0][0];
    }

}