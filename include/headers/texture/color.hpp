
#ifndef PATH_TRACER_COLOR_HPP
#define PATH_TRACER_COLOR_HPP

#include "texture.hpp"

namespace pathtracer{

    class Vector3;

    class Color: public Texture {

    public:

        Color(const Vector3& color);

        // Overriden sample to avoid wasting computation sampling a solid
        // color.
        virtual Vector3 sample(const Vector2& uv) const override;


        virtual Vector3 getPixel(int w, int h) const override;


        virtual Vector3 sampleMipMap(const Vector2& uv, int level) const override;


        virtual void sampleGradient(const Vector2& uv, Vector3& dU, 
            Vector3& dV) const override;

    };

}

#endif