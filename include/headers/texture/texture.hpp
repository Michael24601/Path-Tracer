
#ifndef PATH_TRACER_TEXTURE_HPP
#define PATH_TRACER_TEXTURE_HPP

#include "image.hpp"

namespace pathtracer{

    class Vector2;
    class Vector3;

    // This represents an arbitrary 2D image with n color channels
    class Texture{

    protected:

        // Texture here has 3D data, for color
        Image* image;

        std::vector<Image*> mipmaps;

        // For color textures, we need to turn srgb to linear
        static Vector3 srgbToLinear(const Vector3& color);

        // Maps normal from [0, 1] to [-1, 1] and normalizes
        static Vector3 mapToNormal(const Vector3& color);

        
        void generateMipmaps(bool isNormalMap);


    public:

        // Optionally, for color textures, they can be linearized
        Texture(const std::vector<std::vector<Vector3>>& data, 
            BorderMode borderMode, FilterMode filterMode, 
            bool linearize = false, bool isNormalMap = false,
            bool storeMipmaps = false);

        
        ~Texture(){
            if(image){
                delete image;
            }
        }

        
        inline int getWidth() const{
            return image->getWidth();
        }


        inline int getHeight() const{
            return image->getHeight();
        }


        Vector3 getPixel(int i, int j) const;
        

        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        virtual Vector3 sample(const Vector2& uv) const;


        Vector3 sampleMipMap(const Vector2& uv, int level) const;


        int maxMipMapLevel() const{

            // Each level doubles the number of pixels merged

            // First we pick the larger of the two dimensions
            int size = std::max(image->getWidth(), image->getHeight());

            int level = 0;
            while (size > 1) {
                // We then divide the size by 2, and increment the level
                // each time until it is 1, using bit operators.
                size >>= 1;
                level++;
            }

            return level;
        }

    };

}

#endif