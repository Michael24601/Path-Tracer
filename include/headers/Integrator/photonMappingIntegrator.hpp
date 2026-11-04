
#ifndef PATH_TRACER_PHOTON_MAPPING_INTEGRATOR_HPP
#define PATH_TRACER_PHOTON_MAPPING_INTEGRATOR_HPP

#include "photonMapping/kernel.hpp"
#include "pixelIntegrator.hpp"

namespace pathtracer{

    class PhotonMap;
    class Scene;
    class Camera;
    class ProgressReporter;


    class PhotonMappingIntegrator: public PixelIntegrator{

    private:

        KernelType m_kernelType;
        real m_kernelRadius;
        int m_maxDepth;

        // Used to normalize irradiance
        int m_photonCount;

        // Threshold for photon kdTree
        int m_threshold;

        // Owns the photon map
        PhotonMap* m_map;
        
        // Used for debugging
        std::atomic<long long> m_bounceCount{0};


        // A function that disperses photon in scene and saves
        // them into the map. Returns a new map.
        PhotonMap* tracePhotons(const Scene& scene, ProgressReporter& reporter);


    public:

        PhotonMappingIntegrator(int photonCount, int maxDepth, 
            real kernelRadius, KernelType kernelType, int threshold);


        ~PhotonMappingIntegrator() override;


        // We will overwrite render() function in order to include the
        // photon tracing phase.
        std::vector<std::vector<Vector3>> render(const Scene&, 
            const Camera&, int width, int height,
            ProgressReporter&) override;        
            


        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        Vector3 color(const Ray& ray, const Scene& scene) override;


        void printStatistics(int width, int height) const override;


        // Jittering used for anti-aliasing
        bool jitter() const override{
            return true;
        }


        std::string toString() const override{

            std::string ker;
            switch (m_kernelType){
                case KernelType::UNIFORM: ker = "UNIFORM"; break;
                case KernelType::EPANECHNIKOV: ker = "EPANECHNIKOV"; break;
                default: ker = " ";
            }

            return "PM PHOTON_COUNT=" + std::to_string(m_photonCount)
                + " KERNEL=" + ker + " RADIUS=" + std::to_string(m_kernelRadius)
                + " SPP=" + std::to_string(m_samples);
        }

    };

}

#endif