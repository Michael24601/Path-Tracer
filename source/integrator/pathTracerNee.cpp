
#include "integrator/pathTracerNee.hpp"
#include "bsdf/bsdfSample.hpp"
#include "core/instance.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "core/sceneUtil.hpp"
#include "intersection/intersection.hpp"
#include "light/light.hpp"
#include "light/lightSample.hpp"
#include "math/mathUtil.hpp"
#include "bsdf/bsdf.hpp"
#include "math/constants.hpp"
#include "nextEventEstimation/nextEventEstimation.hpp"
#include "logger.hpp"

namespace pathtracer{


    PathTracerNee::PathTracerNee(int maxDepth) :m_maxDepth{maxDepth} {}


    Vector3 PathTracerNee::color(
        const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        // If the last frame is specular, then no NEE was performed,
        // so we can instead count light bsdf sample contribution
        // if it was hit.
        bool isLastFrameSpecular = false;

        for(int i = 0; i < m_maxDepth; i++){

            Vector3 wo = -currRay.direction();

            // First we intersect the scene
            Intersection it = scene.intersect(currRay);
            // If no hits (we can break or sample envmap)
            if(!it){
                break;
            }

            // If we have an emissive surface, we only consider
            // it on the first bounce, since we are using
            // NEE for every other bounce
            if(it.instance()->emission()){

                if(i == 0 || isLastFrameSpecular){
                    Vector3 emission = it.evaluateEmission(wo);
                    color = color + emission * throughput;
                    break;
                }
                else{
                    // If not first bounce, ignore, since using NEE
                    // (no double count).
                    break;
                }
            }

            // ------------- This next step is NEE -------------

            // We only perform NEE if the current surface is not
            // delta surface (specular)
            if(it.instance()->bsdf()->isSpecular()){
                isLastFrameSpecular = true;
            }
            else{
                isLastFrameSpecular = false;

                NextEventSample s = NextEventEstimation::sample(scene, it);

                // Note that we evaluate, not sample the bsdf, since
                // we already have a wi, so we just need the value
                // and cosine. Also we ignore the pdf it returns since
                // this is the pdf of the bsdf having generated said
                // path (used in MIS for example).
                BsdfSample bsdfEval = it.evaluateBsdf(wo, s.sample.wi());

                if(!bsdfEval.isInvalid() && bsdfEval.cosine() > 0 && s.visibility) {

                    // Contribution includes pdf
                    color = color + s.contribution * bsdfEval.bsdf() 
                        * bsdfEval.cosine() * throughput;
                }
            }

            // ------ This next step is normal pathtracer ------

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
            currRay = Ray(
                it.position() + sample.wi() * SHADOW_EPSILON,
                sample.wi());
        }

        return color;
    }

}