 
#ifndef PATH_TRACER_NEE_PATH_TRACER_HPP
#define PATH_TRACER_NEE_PATH_TRACER_HPP

#include "integrator.hpp"
#include "../core/sceneUtil.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerNEE: public Integrator{

    private:

        int m_maxDepth;
        
    public:

        PathTracerNEE(int maxDepth) : m_maxDepth{maxDepth}{}

        Vector3 color(const Ray& ray, const Scene& scene) const override{

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

                // If we have an emissive surface, we only consider
                // it on the first bounce, since we are using
                // NEE for every other bounce
                if(it.instance()->emission()){

                    if(i == 0){
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


                // Note that NEE can't sample from emissive surfaces,
                // but specifically from lights (emissive surfaces
                // have to be considered an area light explicitly)
                if(scene.lightCount() <= 0){
                    break;
                }

                // First we pick a random light with uniform probability.
                const Light* light = UniformLight::sample(scene);
                // The pdf of choosing this instance
                real pdfInstance = UniformLight::pdf(scene, light);

                // Then we sample the light for a point and radiance,
                // which returns a light sample in world coordinates
                // in solid angle measure.
                LightSample s = light->sample(it.position());
                
                // The visibility term.
                bool visibility = scene.visibility(it.position(), s.position());

                if(s.isValid() && visibility) {

                    // The pdf of choosing this point is the pdf of
                    // choosing the light times the pdf of choosing the point
                    // on the light.
                    real pdfPoint = pdfInstance * s.pdf();

                    // Note that we evaluate, not sample the bsdf, since
                    // we already have a wi, so we just need the value
                    // and cosine. Also we ignore the pdf it returns since
                    // this is the pdf of the bsdf having generated said
                    // path (used in MIS for example).
                    BsdfSample bsdfEval = it.evaluateBsdf(wo, s.wi());
                    if(bsdfEval.cosine() > 0) {
                        Vector3 neeWeight = bsdfEval.bsdf() * bsdfEval.cosine() * (1.0 / pdfPoint);
                        color = color + s.radiance() * throughput * neeWeight;
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
                float p = Util::russianRoulette(throughput);
                if (Random::next() > p){
                    break;
                }

                // And we update the throughput (along with RR probability)
                throughput = throughput * weight * (1.0f / p);
                // The new ray starts at the last intersected point
                // and points towards the new intersected point.
                currRay = Ray(it.position() + sample.wi() * SHADOW_EPSILON, sample.wi());     
            }

            return color;
        }

    };

}

#endif