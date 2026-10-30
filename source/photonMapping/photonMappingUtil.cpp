

#include "photonMapping/photonMappingUtil.hpp"
#include "photonMapping/photonMap.hpp"
#include "core/ray.hpp"
#include "math/constants.hpp"
#include "intersection/intersection.hpp"
#include "core/instance.hpp"
#include "bsdf/bsdf.hpp"
#include "bsdf/bsdfSample.hpp"
#include "photonMapping/photon.hpp"
#include "core/sceneUtil.hpp"
#include "light/light.hpp"
#include "core/scene.hpp"
#include "math/mathUtil.hpp"
#include "core/random.hpp"
#include "light/light.hpp"
#include <omp.h>
#include "logger.hpp"


namespace pathtracer{

    namespace PhotonMapping{

        PhotonMap* tracePhotons(int photonCount, int maxDepth, int threshold,
            const Scene& scene){

            // We create a photon vector for each thread to avoid data race
            std::vector<std::vector<Photon>> threadPhotons(omp_get_max_threads());

            #pragma omp parallel
            {

                std::vector<Photon>& photons = threadPhotons[omp_get_thread_num()];

                // Here we disperse the photons
                #pragma omp for schedule(dynamic, 64)
                for (int photonIndex = 0; photonIndex < photonCount; photonIndex++) {

                    const Light* light = UniformLight::sample(scene);
                    PhotonSample sample = light->samplePhoton(scene);

                    real lightPdf = UniformLight::pdf(scene, light);

                    Vector3 intensity = sample.intensity / lightPdf;
                    Vector3 throughput(1.0);

                    Ray currRay(
                        sample.point + sample.direction * SHADOW_EPSILON,
                        sample.direction);

                    for (int depth = 0; depth < maxDepth; depth++) {

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

                        currRay = Ray(
                            it.position() + bsdfSample.wi() * SHADOW_EPSILON,
                            bsdfSample.wi());
                    }
                }
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
            return new PhotonMap(photons, threshold);
        }

    }
    
}