 
#ifndef PATH_TRACER_PATH_TRACER_SMS_HPP
#define PATH_TRACER_PATH_TRACER_SMS_HPP

#include "integrator.hpp"
#include "../sms/specularManifoldSampling.hpp"
#include "../core/sceneUtil.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerSms: public Integrator{

    private:

        int m_maxDepth;
        
    public:

        SpecularManifoldSampling sms;

        PathTracerSms(int maxDepth, const Scene* scene) 
            : m_maxDepth{maxDepth}, sms(scene){}

        Vector3 color(const Ray& ray, const Scene& scene) override{

            Ray currRay = ray;
            Vector3 color = Vector3(0.0);
            Vector3 throughput = Vector3(1.0);
            const Bsdf* lastBsdf;

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
                    // color = color + emission * throughput;
                    break;
                }


                //---------------------------- SMS -----------------------------


                // If the current surface is neither an emission nor specular,
                // we can try doing SMS.
                if(!it.instance()->bsdf()->isSpecular() && !it.instance()->emission()
                    && it.position().y() < 0.05){

                    // First we sample a point on a light.
                    const Light* light = UniformLight::sample(scene);
                    LightSample s = light->sample(it.position());
                    // The pdf of choosing this instance
                    real pdfSelection = UniformLight::pdf(scene, light);

                    if(s.isValid()){

                        bool success;
                        Vector3 contribution = sms.sample(it, s, wo, 
                            pdfSelection, &scene, success);   
                            
                        // If it worked, we can just return contribution
                        // weighted by throughput.
                        if(success){
                            color = color + contribution * throughput;
                        }
                    }
                }


                // ------ This next step is normal pathtracer ------
                
                // If we didn't hit an emissive surface, we can
                // just update the throughput and move on.
                // Because this is a solid angle estimator, the
                // sampled point is always visible.
                BsdfSample sample = it.sampleBsdf(wo);
                Vector3 weight = sample.weight();

                lastBsdf = it.instance()->bsdf();

                // Russian roulette
                float p = Util::russianRoulette(throughput);
                if (Random::next() > p){
                    break;
                }

                // And we update the throughput (along with RR probability)
                throughput = throughput * weight * (1.0f / (p));
                // The new ray starts at the last intersected point
                // and points towards the new intersected point.
                currRay = Ray(it.position() + sample.wi() * SHADOW_EPSILON, sample.wi());                   
            }
                
            return color;
        }

    };

}

#endif