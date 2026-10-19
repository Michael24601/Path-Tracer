
#ifndef PATH_TRACER_LEAN_MAP_HPP
#define PATH_TRACER_LEAN_MAP_HPP

#include "image.hpp"
#include <vector>

namespace pathtracer{

    class Texture;
    class Matrix2;
    class Vector2;


    class LeanMap{

    private:
        
        // The moments of the lean map, for each level
        std::vector<Image*> m_moments;

    public:

        LeanMap(const Texture* normalMap);

        ~LeanMap(){
            for(int i = 0; i < m_moments.size(); i++){
                if(m_moments[i]){
                    delete m_moments[i];
                }
            }
        }

        // The highest level of the Lean map, which contains only one gaussian
        static int maxLevel(const Texture* normalMap);

        Vector2 mean(const Vector2& uv, int level) const;
        
        Matrix2 covariance(const Vector2& uv, int level) const;
        
    };
}

#endif