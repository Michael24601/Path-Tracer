
#ifndef PATH_TRACER_IMAGE_HPP
#define PATH_TRACER_IMAGE_HPP

#include <cassert>
#include <vector>
#include "config.hpp"

namespace pathtracer{

    class Vector2;
    class Vector3;

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
        // The data is stored in a flat array to avoid pointers to
        // pointer to pointers of arrays...
        std::vector<real> m_data;

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


        const real* getPixel(int i, int j) const;


        // Computes the derivative analytically using the filtering
        // that we have (no central differences).
        void sampleGradient(const Vector2& uv, real* val, real* du, 
            real* dv) const;
        

        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        // This fills a raw array to avoid heap allocating a vector and
        // returning it (since the result has dynamic size).
        void sample(const Vector2& uv, real* out) const;

    };

}

#endif