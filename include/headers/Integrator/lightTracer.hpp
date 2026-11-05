
#ifndef PATH_TRACER_LIGHT_TRACER_HPP
#define PATH_TRACER_LIGHT_TRACER_HPP

#include "lightIntegrator.hpp"
#include <atomic>

namespace pathtracer{

    // A pure path tracer
    class LightTracer: public LightIntegrator{

    private:

        int m_maxDepth;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};

    public:

        LightTracer(int lightPaths, int maxDepth) :
            LightIntegrator(lightPaths), m_maxDepth{maxDepth}{

            assert(maxDepth > 0 && "Max depth must be positive");        
        }


        void tracePath(const Scene& scene, const Camera& camera,
            std::vector<std::vector<Vector3>>& image) override;


        // The only statistic for a path tracer is the bounce count
        void printStatistics(int width, int height) const override;


        std::string toString() const override{
            return "LT PATHS=" + std::to_string(m_lightPaths);
        }
        
    };

}

#endif