
#include "integrator/photonMappingIntegrator.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "intersection/intersection.hpp"
#include "math/vector3.hpp"
#include "bsdf/bsdfSample.hpp"
#include "bsdf/bsdf.hpp"
#include "core/random.hpp"
#include "math/mathUtil.hpp"
#include "core/instance.hpp"
#include "math/constants.hpp"
#include "logger.hpp"


namespace pathtracer{

    
    PhotonMappingIntegrator::PhotonMappingIntegrator(int maxDepth, 
        real kernelRadius, KernelType kernelType) : 
        m_kernelType{kernelType}, m_kernelRadius{kernelRadius}, 
        m_maxDepth{maxDepth}{}


    Vector3 PhotonMappingIntegrator::color(const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        for(int i = 0; i < m_maxDepth; i++){

            Vector3 wo = -currRay.direction();

            // First we intersect the scene
            Intersection it = scene.intersect(currRay);
            // If no hits (we can break or sample envmap)
            if(!it){
                break;
            }

            // If we have an emissive surface, add emission and break.
            if(it.instance()->emission()){
                Vector3 emission = it.evaluateEmission(wo);
                color = color + emission * throughput;
                break;
            }

            //------------------------- PHOTON MAPPING -------------------------

            // This is photon mapping, if done, we can break after it.
            // We can skip it in certain cases, like specular surfaces.

            if(!it.instance()->bsdf()->isSpecular()){

                Vector3 irradiance(0.0);

                // We do density estimation here
                std::vector<Photon> photons;
                m_map->getPhotons(photons, it.position(), m_kernelRadius);
                real radiusSquared = m_kernelRadius * m_kernelRadius;

                for(Photon& photon: photons){
                    // We evaluate BSDF
                    // Recall that in photon mapping, the photons include the BSDF terms
                    // of all previous bounces, but not the current one
                    BsdfSample sample = it.evaluateBsdf(wo, photon.wi);

                    if(sample.isInvalid()){
                        continue;
                    }

                    // We ignore photons on opposite side
                    if(photon.point.geometryNormal().dot(it.geometryNormal()) < 0.0){
                        continue;
                    }

                    real distanceSquared = (photon.point.position() 
                        - it.position()).lengthSquared();

                    // Note, we only use bsdf here, not the whole weight
                    irradiance = irradiance 
                        + photon.weight * sample.bsdf()
                        * Kernel::weight(m_kernelType, distanceSquared, radiusSquared);
                }

                // Then we normalize the total to get the kernel
                irradiance = irradiance / (PI * radiusSquared);
                
                // This is normalization by number of photons
                color = color + irradiance * throughput 
                    / static_cast<real>(m_photonCount);

                // No need to recurse further
                break;
            }

            
            // After this, if we did not photon map, we can do normal bounce
            // and sample direction with bsdf.

            // If we didn't hit an emissive surface, we can
            // just update the throughput and move on.
            // Because this is a solid angle estimator, the
            // sampled point is always visible.
            BsdfSample sample = it.sampleBsdf(wo);
            Vector3 weight = sample.weight();

            // Russian roulette
            float p = Util::russianRoulette(throughput, i);
            if (Random::next() > p){
                break;
            }

            // And we update the throughput (along with RR probability)
            throughput = throughput * weight * (1.0f / p);
            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + sample.wi() * SHADOW_EPSILON,
                sample.wi());
        }

        return color;
    }

}