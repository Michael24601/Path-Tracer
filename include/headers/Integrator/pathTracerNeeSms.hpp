
#ifndef PATH_TRACER_PATH_TRACER_NEE_SMS_HPP
#define PATH_TRACER_PATH_TRACER_NEE_SMS_HPP

#include "pixelIntegrator.hpp"
#include <atomic>

namespace pathtracer{

    class SpecularManifoldSampling;

    // A pure path tracer
    class PathTracerNeeSms: public PixelIntegrator{

    private:

        int m_maxDepth;

        // Owned by this class
        SpecularManifoldSampling* sms;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};

    public:

        PathTracerNeeSms(int samples, int maxDepth, bool useHalfVector, 
            bool biased, bool twoStage, const Scene& scene);


        ~PathTracerNeeSms() override;


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