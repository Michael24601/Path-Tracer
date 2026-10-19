
#include "texture/color.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    Color::Color(const Vector3& color) :
        // Not linearized
        Texture(std::vector<std::vector<Vector3>>{{color}},
            BorderMode::CLAMP, FilterMode::NEAREST, false) {
    }


    Vector3 Color::sample(const Vector2& uv) const {
        // No need to call sample here, since this is just one color
        return getPixel(0, 0);
    }

}