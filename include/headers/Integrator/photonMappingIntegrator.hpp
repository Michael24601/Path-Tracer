
#ifndef PATH_TRACER_PHOTON_MAPPING_INTEGRATOR_HPP
#define PATH_TRACER_PHOTON_MAPPING_INTEGRATOR_HPP

#include "integrator.hpp"
#include "photonMapping/kernel.hpp"
#include "photonMapping/photonMap.hpp"

namespace pathtracer{

    class PhotonMappingIntegrator: public Integrator{

    private:

        KernelType m_kernelType;
        real m_kernelRadius;
        int m_maxDepth;

        // Used to normalize irradiance
        int m_photonCount;

        // Keeps a pointer to a map (does not own it)
        PhotonMap* m_map;

    public:

        PhotonMappingIntegrator(int maxDepth, real kernelRadius, 
            KernelType kernelType);

        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        Vector3 color(const Ray& ray, const Scene& scene) override;

        void setPhotonMap(PhotonMap* map){
            m_map = map;
        }

        void setPhotonCount(int photonCount){
            m_photonCount = photonCount;
        }


        std::string toString() const override{

            std::string ker;
            switch (m_kernelType){
                case KernelType::UNIFORM: ker = "UNIFORM"; break;
                case KernelType::EPANECHNIKOV: ker = "EPANECHNIKOV"; break;
                default: ker = " ";
            }

            return "PM PHOTON_COUNT=" + std::to_string(m_photonCount)
                + " KERNEL=" + ker + " RADIUS=" + std::to_string(m_kernelRadius);
        }

    };

}

#endif