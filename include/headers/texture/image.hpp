
#ifndef PATH_TRACER_IMAGE_HPP
#define PATH_TRACER_IMAGE_HPP

#include <cassert>
#include <vector>
#include "config.hpp"

namespace pathtracer{

    class Vector2;

    enum class BorderMode {
        CLAMP,
        REPEAT,
        MIRROR
    };

    enum class FilterMode {
        NEAREST,
        BILINEAR
    };

    // This represents an arbitrary 2D image with n color channels
    class Image{

    private:

        // 2D texture with n-dimensional data (could be color or other)
        std::vector<std::vector<std::vector<real>>> m_data;

        // Pixel dimensions
        int width, height, channels;

        BorderMode m_borderMode;
        FilterMode m_filterMode;

        // Takes in any uv coordinate, and maps it such that
        // the 0 is at the center of the first pixel, and the
        // 1 is at the center of the last pixel.
        // That means f(0) = 0.5, and f(1) = (W-0.5)
        // (where W is the maximum width or height).
        Vector2 mapToImageSpace(const Vector2& uv) const;


    public:

        // Optionally, for color textures, they can be linearized
        Image(const std::vector<std::vector<std::vector<real>>>& data, 
            BorderMode borderMode, FilterMode filterMode);

        
        inline int getWidth() const{
            return width;
        }


        inline int getHeight() const{
            return height;
        }


        inline int getChannels() const {
            return channels;
        }


        inline BorderMode borderMode() const {
            return m_borderMode;
        }

        inline FilterMode filterMode() const {
            return m_filterMode;
        }


        const std::vector<real>& getPixel(int i, int j) const;
        

        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        std::vector<real> sample(const Vector2& uv) const;

    };

}

#endif