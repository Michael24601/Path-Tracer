

#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "math/matrix2.hpp"
#include "texture/texture.hpp"
#include "texture/leanMap.hpp"
#include "logger.hpp"
#include "math/constants.hpp"
#include <cassert>

namespace pathtracer{

    LeanMap::LeanMap(const Texture* normalMap){

        int maxL = maxLevel(normalMap);
        m_moments.resize(maxL + 1, nullptr);

        // For now, I will only define the very last level
        LOG_WARNING("Only the coarsest level of the Lean Map defined.");

        // The first and second raw moments of the slopes of x and y
        // E[x], E[y], E[xx], E[yy], E[xy]. These are just averages.
        std::vector<real> moments(5, 0.0);

        for(int i = 0; i < normalMap->getHeight(); i++){
            for(int j = 0; j < normalMap->getWidth(); j++){

                // First we compute the slopes
                Vector3 n = normalMap->getPixel(i, j).normalized();
                real z = std::max(n.z(), EPSILON);
                real x = -n.x() / z;
                real y = -n.y() / z;

                moments[0] += x;
                moments[1] += y;
                moments[2] += x*x;
                moments[3] += y*y;
                moments[4] += x*y;
            }
        }

        real size = normalMap->getHeight() * normalMap->getWidth();
        for(int i = 0; i < moments.size(); i++){
            moments[i] /= size;
        }

        std::vector<std::vector<std::vector<real>>> input{{moments}};
        m_moments[maxL] = new Image(input, BorderMode::REPEAT, FilterMode::BILINEAR);
    }


    int LeanMap::maxLevel(const Texture* normalMap){

        // Each level halves the size, rounding up, so non-power-of-two
        // textures still reach a 1x1 level.
        int size = std::max(normalMap->getWidth(), normalMap->getHeight());

        int level = 0;
        while (size > 1) {
            size = (size + 1) >> 1;
            level++;
        }

        return level;
    }


    Vector2 LeanMap::mean(const Vector2& uv, int level) const{

        int maxL = m_moments.size()-1;

        assert(level >= 0 && level <= maxL && "Level is out of bounds");

        if(level != maxL){
            LOG_WARNING("Only max level Lean map is defined, using it as fallback.");
        }

        return Vector2(m_moments[maxL]->getPixel(0, 0)[0],
            m_moments[maxL]->getPixel(0, 0)[1]);
    }
    

    Matrix2 LeanMap::covariance(const Vector2& uv, int level) const{
        
        int maxL = m_moments.size()-1;

        assert(level >= 0 && level <= maxL && "Level is out of bounds");

        if(level != maxL){
            LOG_WARNING("Only max level Lean map is defined, using it as fallback.");
        }

        std::vector<real> moments = m_moments[maxL]->getPixel(0, 0);

        real meanX = moments[0];
        real meanY = moments[1];

        // The covariances are E[xx]-E[x]E[x], and so on for the rest...
        return Matrix2(
            moments[2] - meanX * meanX,
            moments[4] - meanX * meanY,
            moments[4] - meanX * meanY,
            moments[3] - meanY * meanY
        );
    }
        
}
