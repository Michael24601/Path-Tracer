
#ifndef PATH_TRACER_PATH_TRACER_SMS_GUIDED_HPP
#define PATH_TRACER_PATH_TRACER_SMS_GUIDED_HPP

#include "pixelIntegratorGuided.hpp"
#include <cassert>
#include <atomic>

namespace pathtracer{

    class SpecularManifoldSampling;

    class PathTracerSmsGuided: public PixelIntegratorGuided {

    private:

        int m_maxDepth;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};

        // Owned by this class
        SpecularManifoldSampling* sms;

    public:

        PathTracerSmsGuided(int samples, int maxDepth, bool useHalfVector, 
            bool biased, bool twoStage, int newtonIterations, 
            int unbiasedTrials, int biasedTrials, real alpha, int initialSamples, 
            int iterations, real c,  const Scene& scene);


        ~PathTracerSmsGuided() override;


        Vector3 color(const Ray& ray, const Scene& scene) override;


        // The only statistic for a path tracer is the bounce count
        void printStatistics(int width, int height) const override;


        // Jittering used for anti-aliasing
        bool jitter() const override{
            return true;
        }


        std::string toString() const override;


    };

}

#endif