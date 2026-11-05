
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
#include "core/sceneUtil.hpp"
#include "light/light.hpp"
#include "photonMapping/photonMap.hpp"
#include "renderer/progressReporter.hpp"
#include "logger.hpp"
#include <omp.h>


namespace pathtracer{

    
    PhotonMappingIntegrator::PhotonMappingIntegrator(int maxDepth, 
        int photonCount, real kernelRadius, 
        KernelType kernelType, int threshold) : 
        // Only one sample per pixel for a photon mapper
        PixelIntegrator(1), m_photonCount{photonCount},
        m_kernelType{kernelType}, m_kernelRadius{kernelRadius}, 
        m_threshold{threshold}, m_maxDepth{maxDepth}, m_map{nullptr}{}


    PhotonMappingIntegrator::~PhotonMappingIntegrator(){
        if(m_map){
            delete m_map;
        }
    }


    PhotonMap* PhotonMappingIntegrator::tracePhotons(const Scene& scene,
        ProgressReporter& reporter){

        // We create a photon vector for each thread to avoid data race
        std::vector<std::vector<Photon>> threadPhotons(omp_get_max_threads());

        // This is used for logging reports
        reporter.start(m_photonCount, "Photon Tracing");

        #pragma omp parallel
        {

            std::vector<Photon>& photons = threadPhotons[omp_get_thread_num()];

            // Here we disperse the photons
            #pragma omp for schedule(dynamic, 64)
            for (int photonIndex = 0; photonIndex < m_photonCount; photonIndex++) {

                if(g_stopRendering){
                    continue;
                }

                const Light* light = UniformLight::sample(scene);
                EmissionSample sample = light->sampleEmission(scene);

                real lightPdf = UniformLight::pdf(scene, light);

                Vector3 intensity = sample.intensity / lightPdf;
                Vector3 throughput(1.0);

                Ray currRay(sample.point + sample.direction * SHADOW_EPSILON,
                    sample.direction);

                    
                for (int depth = 0; depth < m_maxDepth; depth++) {
                    
                    if (g_stopRendering) {
                        break;
                    }

                    Vector3 wo = -currRay.direction();

                    Intersection it = scene.intersect(currRay);

                    if (!it || it.instance()->emission()) {
                        break;
                    }

                    if (!it.instance()->bsdf()->isSpecular()) {
                        Photon p{intensity, wo, it};
                        photons.push_back(p);
                    }


                    real p = Util::russianRoulette(throughput, depth);
                    if (Random::next() > p) {
                        break;
                    }

                    // Notice that when we sample the bsdf, we have to
                    // tell it we are doing photon mapping (adjoint = true),
                    // since it affects some bsdfs, like refractive specular
                    // lobes.
                    BsdfSample bsdfSample = it.sampleBsdf(wo, true);
                    Vector3 weight = bsdfSample.weight();

                    weight = weight * 
                        Util::shadingNormalAdjoint(bsdfSample.wi(), it, wo);
                    weight = weight / p;

                    throughput = throughput * weight;
                    intensity = intensity * weight;

                    currRay = Ray(it.position() + bsdfSample.wi() * SHADOW_EPSILON,
                        bsdfSample.wi());
                }

                // Everytime a path is done
                reporter.step();
            }
        }

        if (g_stopRendering){
            return nullptr;
        }

        // We then merge the photon vectors from each thread
        // (without copying).
        std::vector<Photon> photons;

        size_t totalCount = 0;
        for (const auto& localPhotons : threadPhotons) {
            totalCount += localPhotons.size();
        }

        photons.reserve(totalCount);

        for (auto& localPhotons : threadPhotons) {
            photons.insert(
                photons.end(),
                std::make_move_iterator(localPhotons.begin()),
                std::make_move_iterator(localPhotons.end()));
        }

        // Once we have our photons, we can create a map
        return new PhotonMap(photons, m_threshold);
    }


    std::vector<std::vector<Vector3>> PhotonMappingIntegrator::render(
        const Scene& scene, const Camera& camera, int width, int height, 
        ProgressReporter& reporter) {

        // First calls trace photons
        if(m_map) delete m_map;
        m_map = tracePhotons(scene, reporter);

        // After that we just call the render function from the super
        // class, which will call the color function from this class.
        return PixelIntegrator::render(scene, camera, width, height, reporter);
    }    


    Vector3 PhotonMappingIntegrator::color(const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        for(int i = 0; i < m_maxDepth; i++){

            Vector3 wo = -currRay.direction();

            // First we intersect the scene
            m_bounceCount++;
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


    void PhotonMappingIntegrator::printStatistics(int width, int height) const {

        long long total = static_cast<long long>(width) * height * m_samples;
        real averageBounceCount = static_cast<real>(m_bounceCount) / total;

        LOG_DEBUG("Bounce count: " + std::to_string(m_bounceCount));
        LOG_DEBUG("Average bounce count: " + std::to_string(averageBounceCount));

        // We also print photon statistics
        LOG_DEBUG("Light samples: " + std::to_string(m_photonCount));
    }


}