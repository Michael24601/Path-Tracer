    
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


    Vector3 Texture::mapToNormal(const Vector3& color){
        Vector3 normal = color * 2.0f - Vector3(1.0f);
        return normal.normalized();
    }


    void Texture::generateMipmaps(bool isNormalMap) {

        // Freeing old levels to avoid leaks
        for (Image* level : mipmaps) {
            delete level;
        }

        mipmaps.clear();

        Image* current = image;

        while (current->getHeight() > 1 || current->getWidth() > 1) {
            int height = std::max(1, (current->getHeight() + 1) / 2);
            int width = std::max(1, (current->getWidth() + 1) / 2);

            std::vector<std::vector<std::vector<real>>> data(
                height, std::vector<std::vector<real>>(width)
            );

            for (int h = 0; h < height; h++) {
                for (int w = 0; w < width; w++) {
                    Vector3 average(0.0f);
                    int count = 0;

                    for (int dh = 0; dh < 2; dh++) {
                        for (int dw = 0; dw < 2; dw++) {
                            int sourceH = 2 * h + dh;
                            int sourceW = 2 * w + dw;

                            if (sourceH >= current->getHeight() ||
                                sourceW >= current->getWidth()) {
                                continue;
                            }

                            std::vector<real> v = current->getPixel(sourceH, sourceW);

                            // Level 0 already holds decoded normals so
                            // no need to redecode them.
                            average = average + Vector3(v[0], v[1], v[2]);
                            count++;
                        }
                    }

                    average = average / static_cast<real>(count);
                    data[h][w] = {average[0], average[1], average[2]};
                }
            }

            Image* next = new Image(data, image->borderMode(), image->filterMode());
            mipmaps.push_back(next);
            current = next;
        }
    }

    Texture::Texture(const std::vector<std::vector<Vector3>>& data, 
        BorderMode borderMode, FilterMode filterMode, bool linearize,
        bool isNormalMap, bool storeMipMaps) {

        std::vector<std::vector<std::vector<real>>> rawData(data.size());
        
        for(int i = 0; i < data.size(); i++){
            rawData[i].resize(data[i].size());

            for(int j = 0; j < data[i].size(); j++){

                Vector3 color = data[i][j];

                if(isNormalMap){
                    color = mapToNormal(color);
                }
                // Colors must be linearized
                else if(linearize){
                    color = srgbToLinear(color);
                }

                rawData[i][j] = std::vector<real>{color[0], color[1], color[2]};
            }
        }

        // This checks for ragged input
        image = new Image(rawData, borderMode, filterMode);

        if(storeMipMaps){
            generateMipmaps(isNormalMap);
        }

    }


    Vector3 Texture::getPixel(int i, int j) const{
        std::vector<real> p = image->getPixel(i, j);
        return Vector3(p[0], p[1], p[2]);
    }
        

    Vector3 Texture::sample(const Vector2& uv) const{
        std::vector<real> p = image->sample(uv);
        return Vector3(p[0], p[1], p[2]);
    }



    Vector3 Texture::sampleMipMap(const Vector2& uv, int level) const {
        
        // If the level is wrong, or we don't store any mipmaps
        if (level <= 0 || mipmaps.empty()) {
            // Then we just return a normal sample
            return sample(uv);
        }

        level = std::min(level, static_cast<int>(mipmaps.size()));

        std::vector<real> p = mipmaps[level - 1]->sample(uv);
        return Vector3(p[0], p[1], p[2]).normalized();
    }

}