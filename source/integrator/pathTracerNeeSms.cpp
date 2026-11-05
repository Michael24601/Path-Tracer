
#include "integrator/pathTracerNeeSms.hpp"
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
#include "sms/specularManifoldSampling.hpp"
#include "nextEventEstimation/nextEventEstimation.hpp"
#include "bsdf/bsdf.hpp"
#include "math/constants.hpp"
#include "logger.hpp"


namespace pathtracer{


    PathTracerNeeSms::PathTracerNeeSms(int samples, int maxDepth, 
        bool useHalfVector, bool biased, bool twoStage, const Scene& scene) : 
        PixelIntegrator(samples), m_maxDepth{maxDepth}, sms{nullptr}{

        assert(maxDepth > 0 && "Max depth must be positive");    
        
        sms = new SpecularManifoldSampling(scene, useHalfVector, biased, twoStage);
    }


    PathTracerNeeSms::~PathTracerNeeSms(){
        if(sms){
            delete sms;
        }
    }


    Vector3 PathTracerNeeSms::color(const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        // saves the last sms sample bounce index
        const Instance* lastInst = nullptr;

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
            // If last hit was a specular (can cast caustic), we ignore it,
            // since it would have double counted SMS sample.
            // And if last hit is not specular, we also ignore it,
            // since it was sampled by NEE.
            // So we only consider first hit.
            if(it.instance()->emission()){
                if(i== 0){
                    Vector3 emission = it.evaluateEmission(wo);
                    color = color + emission * throughput;
                }
                break;
            }

            lastInst = it.instance();

            //---------------------------- SMS -----------------------------

            // If the current surface is neither an emission nor specular,
            // we can try doing SMS.
            if(!it.instance()->bsdf()->isSpecular() && !it.instance()->emission() &&
                it.instance()->isCausticReceiver()){

                Vector3 contribution = sms->sample(it, wo, &scene);

                // If it worked, we can just return contribution
                // weighted by throughput. Otherwise it is 0.0 anyway.
                color = color + contribution * throughput;
            }


            // --------------------- This next step is NEE ---------------------

            // We only perform NEE if the current surface is not
            // delta surface (specular).
            // This is both because NEE samples have a 0 probability
            // of sampling the correct direction, and because
            // it would double count SMS.
            if(!it.instance()->bsdf()->isSpecular()){

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
            real p = Util::russianRoulette(throughput, i);

            if (Random::next() > p){
                break;
            }

            // And we update the throughput (along with RR probability)
            throughput = throughput * weight * (1.0f / (p));

            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + sample.wi() * SHADOW_EPSILON,
                sample.wi());

        }

        return color;
    }



    void PathTracerNeeSms::printStatistics(int width, int height) const {

        long long total = static_cast<long long>(width) * height * m_samples;
        real averageBounceCount = static_cast<real>(m_bounceCount) / total;

        LOG_DEBUG("Bounce count: " + std::to_string(m_bounceCount));
        LOG_DEBUG("Average bounce count: " + std::to_string(averageBounceCount));

        // We also print sms statistics
        sms->printStatistics();
    }


    std::string PathTracerNeeSms::toString() const{
        return "PT NEE " + sms->toString() + " SPP=" + std::to_string(m_samples);
    }


}