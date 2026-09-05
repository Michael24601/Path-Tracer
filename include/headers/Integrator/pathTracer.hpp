
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

                // First bounce is just direct light
                Intersection it = scene.intersect(currRay);

                if(!it){
                    color = Vector3(0.0);
                }
                else{
                    color = it.evaluateEmission(-currRay.direction());

                    // The surface point we bounce on at each iteration
                    SurfacePoint sp = it;
                    
                    for(int i = 1; i < m_depth; i++){

                        Vector3 wo = -currRay.direction();

                        // ------ This next step is normal pathtracer ------
                    
                        MonteCarloEstimator est = MonteCarlo::sampleSolidAngle(wo, sp, scene);

                        Vector3 le;

                        if(est.visible()){
                            // If not visible, it means the sample is 
                            // invalid for some reason, so we will quit
                            // after this sample.

                            // Note that the new radiance is multiplied by the
                            // throughput, including the weight we get
                            // form the intersection.
                            le = est.radiance() * throughput * est.weight();
                        }
                        else{
                            le = Vector3(0.0) * throughput * est.weight();
                        }

                        // Finally, we add li and le to the color
                        color = color + le;

                        if(!est.visible() || 
                            est.it().instance()->emission() != nullptr){
                            // If we couldnt sample a next point, we can break,
                            // as we can no longer trace a path.
                            // Or if we hit an emissive surface.
                            break;
                        }

                        // And we update the throughput
                        throughput = throughput * est.weight();
                        // The new ray starts at the last intersected point
                        // and points towards the new intersected point.
                        currRay = Ray(est.it().position(), est.wi());
                        // And the new intersected point is the one we sampled.
                        sp = est.it();
                    }
                }

                averageLe = (averageLe * k + color) * (1.0 / (k+1.0));

            }

            return averageLe;
        }

    };

}

#endif