
#ifndef PATH_TRACER_PATH_TRACER_GUIDED_HPP
#define PATH_TRACER_PATH_TRACER_GUIDED_HPP

#include "pixelIntegratorGuided.hpp"
#include <cassert>

namespace pathtracer{

    class PathTracerGuided: public PixelIntegratorGuided {

    private:

        int m_maxDepth;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};

    public:

        PathTracerGuided(int samples, real alpha, int initialSamples, 
            int iterations, real c, int maxDepth) :
            PixelIntegratorGuided(samples, alpha, initialSamples,
                iterations, c), m_maxDepth{maxDepth}{

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
            return "PT GUIDED ALPHA=" + std::to_string(m_alpha) 
                + " TRAINING_IT=" + std::to_string(m_iterations) 
                + " SPP=" + std::to_string(m_samples);
        }


    };

}

#endif