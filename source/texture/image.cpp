
#include "texture/image.hpp"
#include "math/mathUtil.hpp"
#include "texture/imageUtil.hpp"
#include "math/vector2.hpp"
#include "math/vector2i.hpp"
#include "math/vector3.hpp"


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
        m_borderMode{borderMode},
        m_filterMode{filterMode} {

        assert((data.size() > 0 && data[0].size() > 0)
            && "Image is not filled");

        height = data.size();
        width = data[0].size();
        channels = data[0][0].size();

        // We then flatten the data here
        m_data.resize(size_t(width) * height * channels);

        for (int h = 0; h < height; h++) {
            assert(static_cast<int>(data[h].size()) == width && "ragged row");

            for (int w = 0; w < width; w++) {
                assert(static_cast<int>(data[h][w].size()) == channels && "channel mismatch");

                int baseIndex = (h * width + w) * channels;
                for (int c = 0; c < channels; c++) {
                    m_data[baseIndex + c] = data[h][w][c];
                }
            }
        }

    }


    void Image::sample(const Vector2& uv, real* out) const {

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
        if(m_filterMode == FilterMode::NEAREST){
            Filter::nearest(decimal, floor, ceiling, m_data.data(), 
                width, channels, out);
        }
        else if(m_filterMode == FilterMode::BILINEAR){
            Filter::bilinear(decimal, floor, ceiling, m_data.data(), 
                width, channels, out);
        }
    }


    void Image::sampleGradient(const Vector2& uv, real* val, 
        real* du, real* dv) const {

        const int w = getWidth();
        const int h = getHeight();

        switch (m_filterMode) {

        case FilterMode::BILINEAR: {

            Vector2 imageUv = mapToImageSpace(uv);

            Vector2i floor = Util::floor(imageUv);
            Vector2i ceiling = Util::ceiling(imageUv);
            Vector2 decimal = imageUv - Vector2(floor.x(), floor.y());

            if (m_borderMode == BorderMode::CLAMP) {
                floor = Border::clamp(floor, w, h);
                ceiling = Border::clamp(ceiling, w, h);
            }
            else if (m_borderMode == BorderMode::REPEAT) {
                floor = Border::repeat(floor, w, h);
                ceiling = Border::repeat(ceiling, w, h);
            }
            else if (m_borderMode == BorderMode::MIRROR) {
                floor = Border::mirror(floor, w, h);
                ceiling = Border::mirror(ceiling, w, h);
            }

            const real* p00 = getPixel(floor.y(), floor.x());
            const real* p10 = getPixel(floor.y(), ceiling.x());
            const real* p01 = getPixel(ceiling.y(), floor.x());
            const real* p11 = getPixel(ceiling.y(), ceiling.x());

            Filter::bilinear(decimal, floor, ceiling, m_data.data(), w, channels, val);

            // Convert from pixel space to uv space (chain rule,
            // and this is a scaling operation, so derivative is a
            // factor of thw width or height).
            real sx = real(w - 1);
            real sy = -real(h - 1);

            for (int c = 0; c < channels; c++) {
                real dx = (p10[c] - p00[c]) * (1 - decimal.y())
                    + (p11[c] - p01[c]) * decimal.y();
                real dy = (p01[c] - p00[c]) * (1 - decimal.x())
                    + (p11[c] - p10[c]) * decimal.x();

                du[c] = dx * sx;
                dv[c] = dy * sy;
            }
            return;
        }
        case FilterMode::NEAREST: {
            // Not differentiable
            // Could maybe use central differences in the future
            sample(uv, val);
            for (int c = 0; c < channels; c++) {
                du[c] = 0.0f;
                dv[c] = 0.0f;
            }
            return;
        }
        }
    }


    const real* Image::getPixel(int h, int w) const {
        assert((h >= 0 && h < height) && (w>= 0 && w< width) &&
            "Dimensions don't match the texture's");

        return &m_data[(size_t(h) * width + w) * channels];
    }


}