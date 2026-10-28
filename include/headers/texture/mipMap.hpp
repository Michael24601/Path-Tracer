
#ifndef PATH_TRACER_MIPMAP_HPP
#define PATH_TRACER_MIPMAP_HPP

#include "image.hpp"
#include <vector>

namespace pathtracer{

    class Vector2;

    class MipMap{

    private:
        
        // The levels of the mipmap
        std::vector<Image> m_levels;

    public:

        MipMap(const Image& base);

        int maxLevel() const;
        
        std::vector<real> getPixel(int h, int w, int level) const;

        std::vector<real> sample(const Vector2& uv, int level) const;

        const Image& level(int i) const;

        
    };
}

#endif