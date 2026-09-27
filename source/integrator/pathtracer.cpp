
#include "integrator/pathTracer.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "intersection/intersection.hpp"
#include "math/vector3.hpp"
#include "bsdf/bsdfSample.hpp"
#include "core/random.hpp"
#include "math/mathUtil.hpp"
#include "core/instance.hpp"
#include "math/constants.hpp"


namespace pathtracer{

    
    PathTracer::PathTracer(int maxDepth) : m_maxDepth{maxDepth} {}


    Vector3 PathTracer::color(
        const Ray& ray, const Scene& scene) {

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

            // If we have an emissive surface, add emission and break
            if(it.instance()->emission()){
                Vector3 emission = it.evaluateEmission(wo);
                color = color + emission * throughput;
                break;
            }

            // ------ This next step is normal pathtracer ------

            // If we didn't hit an emissive surface, we can
            // just update the throughput and move on.
            // Because this is a solid angle estimator, the
            // sampled point is always visible.
            BsdfSample sample = it.sampleBsdf(wo);
            Vector3 weight = sample.weight();

            // Russian roulette
            float p = Util::russianRoulette(throughput);
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