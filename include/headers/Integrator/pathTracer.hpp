 
#ifndef PATH_TRACER_PATH_TRACER_HPP
#define PATH_TRACER_PATH_TRACER_HPP

#include "integrator.hpp"
#include "monteCarlo.hpp"

namespace pathtracer{

    class PathTracer: public Integrator{

    private:

        int m_depth;
        int m_samples;
        
    public:

        PathTracer(int depth, int samples) : m_depth{depth}, 
            m_samples{samples}{}

        Vector3 color(const Ray& ray, const Scene& scene) const override{

            Vector3 averageLe(0.0);

            for(int k = 0; k < m_samples; k++){

                Ray currRay = ray;
                Vector3 color = Vector3(0.0);
                Vector3 throughput = Vector3(1.0);

                for(int i = 1; i < m_depth; i++){

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
                    currRay = Ray(it.position(), sample.wi());                
                }
                
                averageLe = (averageLe * k + color) * (1.0 / (k+1.0));

            }

            return averageLe;
        }

    };

}

#endif