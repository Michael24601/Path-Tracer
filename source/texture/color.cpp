
#include "texture/color.hpp"

namespace pathtracer{

    Color::Color(const Vector3& color) :
        Texture(std::vector<std::vector<Vector3>>{{color}},
            BorderMode::CLAMP, FilterMode::NEAREST) {
    }


    Vector3 Color::sample(const Vector2& uv) const {
        return m_data[0][0];
    }

}