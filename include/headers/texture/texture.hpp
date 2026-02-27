
#ifndef PATH_TRACER_TEXTURE_HPP
#define PATH_TRACER_TEXTURE_HPP

#include "vector3.hpp"
#include "vector2.hpp"
#include "textureUtil.hpp"
#include "mathUtil.hpp"

namespace pathtracer{

    class Texture{

    private:

        enum class BorderMode {
            CLAMP,
            REPEAT,
            MIRROR
        };


        enum class FilterMode {
            NEAREST,
            BILINEAR
        };
        

        std::vector<std::vector<Vector3>> m_data;
        int width, height;
        BorderMode m_borderMode;
        FilterMode m_filterMode;


        // Takes in any uv coordinate, and maps it such that
        // the 0 is at the center of the first pixel, and the
        // 1 is at the center of the last pixel.
        // That means f(0) = 0.5, and f(1) = (W-0.5)
        // (where W is the maximum width or height).
        Vector2 mapToImageSpace(const Vector2& uv) const{
            // Since textures often start at the top left, but uv
            // originates at the bottom left, we have to flip the
            // y axis.
            Vector2 output(uv.x() * (width-1) + 0.5,
                (1.0 - uv.y()) * (height-1) + 0.5);
            return output;
        }

    public:

        Texture(const std::vector<std::vector<Vector3>>& data,
            BorderMode borderMode, FilterMode filterMode) :
            m_data{data}, m_borderMode{borderMode}, 
            m_filterMode{filterMode}{

                assert((data.size() > 0 && data[0].size() > 0) 
                    && "Texture is not filled");
                
                width = data[0].size();
                height = data.size();
            }


        // Samples the texture according to the set modes. The given
        // uv coordinate does not necessarily need to span between
        // 0 and 1, as the borderMode will handle it, nor does it
        // need to lie on any particular pixel center as the filterMode
        // will handle that.
        Vector3 sample(const Vector2& uv) const{

            // First we precompute these values
            Vector2 imageUv = mapToImageSpace(uv);
            Vector2i floor = Util::floor(imageUv);
            Vector2i ceiling = Util::ceiling(imageUv);
            Vector2 decimal = imageUv - Vector2(floor.x(), floor.y());

            // We then apply border handling on the integer vectors
            if(m_borderMode == BorderMode::CLAMP){
                Border::clamp(floor, width, height);
                Border::clamp(ceiling, width, height);
            }
            else if(m_borderMode == BorderMode::REPEAT){
                Border::repeat(floor, width, height);
                Border::repeat(ceiling, width, height);
            }
            else if(m_borderMode == BorderMode::MIRROR){
                Border::mirror(floor, width, height);
                Border::mirror(ceiling, width, height);
            }

            // Then we apply filtering
            Vector3 color;
            if(m_filterMode == FilterMode::NEAREST){
                color = Filter::nearest(decimal, floor, ceiling, m_data);
            }
            else if(m_filterMode == FilterMode::BILINEAR){
                color = Filter::bilinear(decimal, floor, ceiling, m_data);
            }

            return color;
        }

    };

}

#endif