    
#include "texture/texture.hpp"
#include "math/vector3.hpp"
#include "math/vector2.hpp"
    
namespace pathtracer{

    Vector3 Texture::srgbToLinear(const Vector3& color) {

        Vector3 result = color;

        for (int i = 0; i < 3; i++) {
            if (result[i] <= 0.04045){
                result[i] /= 12.92;
            }
            else{
                result[i] = std::pow((result[i] + 0.055) / 1.055, 2.4);
            }
        }

        return result;
    }


    Texture::Texture(const std::vector<std::vector<Vector3>>& data, 
        BorderMode borderMode, FilterMode filterMode, bool linearize) {

        std::vector<std::vector<std::vector<real>>> rawData(data.size());
        
        for(int i = 0; i < data.size(); i++){
            rawData[i].resize(data[i].size());

            for(int j = 0; j < data[i].size(); j++){

                Vector3 color = data[i][j];

                // Colors must be linearized
                if(linearize){
                    color = srgbToLinear(color);
                }

                rawData[i][j] = std::vector<real>{color[0], color[1], color[2]};
            }
        }

        // This checks for ragged input
        image = new Image(rawData, borderMode, filterMode);

    }


    void Texture::setPixel(int i, int j, const Vector3& color){
        std::vector<real> pixel{color[0], color[1], color[2]};
        image->setPixel(i, j, pixel);
    }


    Vector3 Texture::getPixel(int i, int j) const{
        std::vector<real> p = image->getPixel(i, j);
        return Vector3(p[0], p[1], p[2]);
    }
        

    Vector3 Texture::sample(const Vector2& uv) const{
        std::vector<real> p = image->sample(uv);
        return Vector3(p[0], p[1], p[2]);
    }

}