
#include "texture/image.hpp"
#include "math/mathUtil.hpp"
#include "texture/imageUtil.hpp"
#include "math/vector2.hpp"
#include "math/vector2i.hpp"


namespace pathtracer{


    Vector2 Image::mapToImageSpace(const Vector2& uv) const {
        // Since textures often start at the top left, but uv
        // originates at the bottom left, we have to flip the
        // y axis.
        Vector2 output(uv.x() * (width - 1) + 0.5,
            (1.0 - uv.y()) * (height - 1) + 0.5);
        return output;
    }



    Image::Image(const std::vector<std::vector<std::vector<real>>>& data, 
        BorderMode borderMode, FilterMode filterMode) :
        m_data{data},
        m_borderMode{borderMode},
        m_filterMode{filterMode} {

        assert((data.size() > 0 && data[0].size() > 0)
            && "Image is not filled");

        width = data[0].size();
        height = data.size();
        channels = data[0][0].size();

        // This ensures they have the same dimensions everywhere
        for (const auto& row : data) {
            assert(static_cast<int>(row.size()) == width && "ragged row");
            for (const auto& pixel : row){
                assert(static_cast<int>(pixel.size()) == channels && "channel mismatch");
            }
        }

    }


    std::vector<real> Image::sample(const Vector2& uv) const {

        // First we precompute these values
        Vector2 imageUv = mapToImageSpace(uv);
        Vector2i floor = Util::floor(imageUv);
        Vector2i ceiling = Util::ceiling(imageUv);
        Vector2 decimal = imageUv - Vector2(floor.x(), floor.y());

        // We then apply border handling on the integer vectors
        if(m_borderMode == BorderMode::CLAMP){
            floor = Border::clamp(floor, width, height);
            ceiling = Border::clamp(ceiling, width, height);
        }
        else if(m_borderMode == BorderMode::REPEAT){
            floor = Border::repeat(floor, width, height);
            ceiling = Border::repeat(ceiling, width, height);
        }
        else if(m_borderMode == BorderMode::MIRROR){
            floor = Border::mirror(floor, width, height);
            ceiling = Border::mirror(ceiling, width, height);
        }

        // Then we apply filtering
        std::vector<real> color;
        if(m_filterMode == FilterMode::NEAREST){
            color = Filter::nearest(decimal, floor, ceiling, m_data);
        }
        else if(m_filterMode == FilterMode::BILINEAR){
            color = Filter::bilinear(decimal, floor, ceiling, m_data);
        }

        return color;
    }


    // Sets texture
    void Image::setPixel(int h, int w, const std::vector<real>& color){

        assert((h >= 0 && h < height) && (w >= 0 && w < width) &&
            "Dimensions don't match the texture's");

        assert(color.size() == channels);

        m_data[h][w] = color;
    }


    const std::vector<real>& Image::getPixel(int h, int w) const {
        assert((h >= 0 && h < height) && (w >= 0 && w < width) &&
            "Dimensions don't match the texture's");

        return m_data[h][w];
    }


}