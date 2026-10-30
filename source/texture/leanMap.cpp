

#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "math/matrix2.hpp"
#include "texture/texture.hpp"
#include "texture/leanMap.hpp"
#include "logger.hpp"
#include "math/constants.hpp"
#include <cassert>

namespace pathtracer{

    namespace LeanMap{

        MipMap* constructLeanMap(const Texture* normalMap){

            const int height = normalMap->getHeight();
            const int width = normalMap->getWidth();

            std::vector<std::vector<std::vector<real>>> data(
                height, std::vector<std::vector<real>>(width)
            );

            for(int i = 0; i < height; i++){
                for(int j = 0; j < width; j++){

                    // First we compute the slopes
                    Vector3 n = normalMap->getPixel(i, j).normalized();
                    real z = std::max(n.z(), EPSILON);
                    real x = -n.x() / z;
                    real y = -n.y() / z;

                    // We compute the first and second moments E[x], E[y], 
                    // E[xx], E[yy], E[xy]. These are then averaged
                    // for the mipmap.
                    data[i][j] = {x, y, x*x, y*y, x*y};
                }
            }

            Image image(data, BorderMode::REPEAT, FilterMode::BILINEAR);
            return new MipMap(image);
        }

        
        Vector2 mean(const MipMap* map, const Vector2& uv, int level){
            real m[5];
            map->sample(uv, level, m);
            return Vector2(m[0], m[1]);
        }


        Matrix2 covariance(const MipMap* map, const Vector2& uv, int level){

            real m[5];
            map->sample(uv, level, m);
            
            // The covariances are E[xx]-E[x]E[x], and so on for the rest...
            real vxx = m[2] - m[0] * m[0];
            real vyy = m[3] - m[1] * m[1];
            real vxy = m[4] - m[0] * m[1];

            // We can clamp the variance since they can't be negative
            vxx = std::max(vxx, static_cast<real>(0));
            vyy = std::max(vyy, static_cast<real>(0));

            return Matrix2(vxx, vxy, vxy, vyy);
        }
    }
            
}
