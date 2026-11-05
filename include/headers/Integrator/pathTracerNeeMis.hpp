
#ifndef PATH_TRACER_PATH_TRACER_NEE_MIS_HPP
#define PATH_TRACER_PATH_TRACER_NEE_MIS_HPP

#include "pixelIntegrator.hpp"
#include <atomic>


namespace pathtracer{

    // A pure path tracer
    class PathTracerNeeMis: public PixelIntegrator{

    private:

        int m_maxDepth;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};

    public:

        PathTracerNeeMis(int samples, int maxDepth) :
            PixelIntegrator(samples), m_maxDepth{maxDepth}{

            assert(maxDepth > 0 && "Max depth must be positive");        
        }


        Vector3 color(const Ray& ray, const Scene& scene) override;


        // The only statistic for a path tracer is the bounce count
        void printStatistics(int width, int height) const override;


        // Jittering used for anti-aliasing
        bool jitter() const override{
            return true;
        }


        std::string toString() const override{
            return "PT NEE MIS SPP=" + std::to_string(m_samples);
        }
        
    };

}

#endif