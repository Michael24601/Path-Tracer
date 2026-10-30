    
#include "texture/texture.hpp"
#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "math/constants.hpp"

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


    Vector3 Texture::outputSample(const real* p) const {
        Vector3 v(p[0], p[1], p[2]);
        return m_isNormalMap ? v.normalized() : v;
    }


    Image Texture::buildImage(const std::vector<std::vector<Vector3>>& data,
        BorderMode borderMode, FilterMode filterMode,
        bool linearize, bool isNormalMap) {

        std::vector<std::vector<std::vector<real>>> rawData(data.size());

        for(size_t i = 0; i < data.size(); i++){
            rawData[i].resize(data[i].size());

            for(size_t j = 0; j < data[i].size(); j++){

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

        return Image(rawData, borderMode, filterMode);
    }


    Texture::Texture(const std::vector<std::vector<Vector3>>& data, 
        BorderMode borderMode, FilterMode filterMode, bool linearize,
        bool isNormalMap) : m_isNormalMap{isNormalMap}, mipmap(
            buildImage(data, borderMode, filterMode, linearize, isNormalMap
        )) {}


    void Texture::sampleGradient(const Vector2& uv,
        Vector3& dndu, Vector3& dndv) const {

        real val[3], gu[3], gv[3];
        mipmap.level(0).sampleGradient(uv, val, gu, gv);

        Vector3 v(val[0], val[1], val[2]);
        Vector3 dvu(gu[0], gu[1], gu[2]);
        Vector3 dvv(gv[0], gv[1], gv[2]);

        if (!m_isNormalMap) {
            // If not a normal map, we just use the derivatives as is
            // and don't need to normzlied.
            dndu = dvu;
            dndv = dvv;
            return;
        }

        real len = v.length();

        if (len > EPSILON) {
            // This code turns derivatives of v into derivatives
            // of normalized v.
            v = v / len;
            dndu = (dvu - v * v.dot(dvu)) / len;
            dndv = (dvv - v * v.dot(dvv)) / len;
        }
        else {
            dndu = Vector3(0.0f);
            dndv = Vector3(0.0f);
        }
    }


    Vector3 Texture::getPixel(int i, int j) const{
        const real* p = mipmap.level(0).getPixel(i, j);
        return outputSample(p);
    }
        

    Vector3 Texture::sample(const Vector2& uv) const{
        real p[3];
        mipmap.level(0).sample(uv, p);
        return outputSample(p);
    }



    Vector3 Texture::sampleMipMap(const Vector2& uv, int level) const {
        real p[3];
        mipmap.level(level).sample(uv, p);
        return outputSample(p);
    }

}