
#ifndef PATH_TRACER_TEXTURE_HPP
#define PATH_TRACER_TEXTURE_HPP

#include "mipMap.hpp"

namespace pathtracer{

    class Vector2;
    class Vector3;

    // This represents an arbitrary 2D image with n color channels
    class Texture{

    protected:

        // Uses 3D images for RGB
        MipMap mipmap;

        bool m_isNormalMap;

        // For color textures, we need to turn srgb to linear
        static Vector3 srgbToLinear(const Vector3& color);

        // Maps normal from [0, 1] to [-1, 1] and normalizes
        static Vector3 mapToNormal(const Vector3& color);


        Vector3 outputSample(const std::vector<real>& p) const;

        
        static Image buildImage(const std::vector<std::vector<Vector3>>& data,
            BorderMode borderMode, FilterMode filterMode,
            bool linearize, bool isNormalMap);


    public:

        // Optionally, for color textures, they can be linearized
        Texture(const std::vector<std::vector<Vector3>>& data, 
            BorderMode borderMode, FilterMode filterMode, 
            bool linearize = false, bool isNormalMap = false);

        
        inline int getWidth() const{
            return mipmap.level(0).getWidth();
        }


        inline int getHeight() const{
            return mipmap.level(0).getHeight();
        }


        virtual Vector3 getPixel(int i, int j) const;
        

        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        virtual Vector3 sample(const Vector2& uv) const;


        virtual Vector3 sampleMipMap(const Vector2& uv, int level) const;


        virtual void sampleGradient(const Vector2& uv,
            Vector3& dU, Vector3& dV) const;


        int maxMipMapLevel() const{
            return mipmap.maxLevel();
        }

    };

}

#endif