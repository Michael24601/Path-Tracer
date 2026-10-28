
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
        return Texture::getPixel(0, 0);
    }


    Vector3 Color::getPixel(int h, int w) const {
        return Texture::getPixel(0, 0);
    }


    Vector3 Color::sampleMipMap(const Vector2& uv, int level) const {
        return Texture::getPixel(0, 0);
    }


    void Color::sampleGradient(const Vector2& uv, Vector3& dU, Vector3& dV) const{
        // No gradient
        dU = Vector3(0.0);
        dV = Vector3(0.0);
    }

}