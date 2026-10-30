#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "texture/mipMap.hpp"
#include "logger.hpp"
#include "math/constants.hpp"
#include <cassert>

namespace pathtracer{

    MipMap::MipMap(const Image& base){

        // Level 0 is the original image.
        m_levels.push_back(base);

        int currentLevel = 0;

        while (m_levels[currentLevel].getHeight() > 1 ||
            m_levels[currentLevel].getWidth() > 1) {
            const Image& current = m_levels[currentLevel];

            int height = std::max(1, (current.getHeight() + 1) / 2);
            int width = std::max(1, (current.getWidth() + 1) / 2);
            int channels = current.getChannels();

            std::vector<std::vector<std::vector<real>>> data(
                height, std::vector<std::vector<real>>(width)
            );

            for (int h = 0; h < height; h++) {
                for (int w = 0; w < width; w++) {
                    std::vector<real> average(channels, static_cast<real>(0));
                    int count = 0;

                    for (int dh = 0; dh < 2; dh++) {
                        for (int dw = 0; dw < 2; dw++) {
                            int sourceH = 2 * h + dh;
                            int sourceW = 2 * w + dw;

                            if (sourceH >= current.getHeight() ||
                                sourceW >= current.getWidth()) {
                                continue;
                            }

                            const real* v = current.getPixel(sourceH, sourceW);

                            for (int c = 0; c < channels; c++) {
                                average[c] += v[c];
                            }

                            count++;
                        }
                    }

                    for (int c = 0; c < channels; c++) {
                        average[c] /= static_cast<real>(count);
                    }

                    data[h][w] = std::move(average);
                }
            }

            m_levels.push_back(
                Image(data, base.borderMode(), base.filterMode())
            );

            currentLevel++;
        }
    }



    int MipMap::maxLevel() const{
        return static_cast<int>(m_levels.size()) - 1;
    }



    void MipMap::sample(const Vector2& uv, int level, real* out) const{
        int maxL = m_levels.size()-1;
        assert(level >= 0 && level <= maxL && "Level is out of bounds");
        m_levels[level].sample(uv, out);
    }
    

    const real* MipMap::getPixel(int h, int w, int level) const{

        int maxL = m_levels.size()-1;
        assert(level >= 0 && level <= maxL && "Level is out of bounds");

        return m_levels[level].getPixel(h, w);
    }
    

    const Image& MipMap::level(int i) const {
        assert(i >= 0 && i < static_cast<int>(m_levels.size()) 
            && "Level is out of bounds");
        return m_levels[i];
    }
        
}