
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
        
        void sample(const Vector2& uv, int level, real* out) const;

        const real* getPixel(int h, int w, int level) const;

        const Image& level(int i) const;

        
    };
}

#endif