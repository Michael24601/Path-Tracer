
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
                color = color + it.evaluateEmission(-currRay.direction());

                // The surface point we bounce on at each iteration
                SurfacePoint sp = it;
                
                for(int i = 1; i < m_depth; i++){

                    Vector3 wo = -currRay.direction();

                    // ------ This first step is NEE -------

                    MonteCarloLightSample nee = MonteCarlo::sampleLight(wo, sp, scene);

                    Vector3 li;

                    // If not visible, we just return 0 for li
                    if(!nee.visible()){
                        li = Vector3(0.0);
                    }

                    // Note that the new radiance is multiplied by the
                    // throughput, including the weight we get
                    // form the intersection.
                    li = nee.radiance() * throughput * nee.weight();

                    // ------ This next step is normal pathtracer ------
                
                    MonteCarloEstimator est = MonteCarlo::sampleSolidAngle(wo, sp, scene);

                    if(!est.visible()){
                        // This means it is either invisible or the
                        // sample is invalid for some reason, so we
                        // quit.
                        break;
                    }

                    // Note that the new radiance is multiplied by the
                    // throughput, including the weight we get
                    // form the intersection.
                    Vector3 le = est.radiance() * throughput * est.weight();
                    

                    // ------ Next up is MIS to blend Li and Le ------

                    // Le is weighted by MIS if the intersected
                    // point is an area light, and therefore intersectable
                    // by NEE.
                    if(est.it().instance()->light() != nullptr){
                        // Probability of intersecting instance
                        // using pathtracing (in solid angles).
                        real pLe = est.pdf();
                        // Probability of intersecting it as an area
                        // light in nee.
                        real pLi = MonteCarlo::evaluateLightPdf(sp.position(), 
                            est.it().instance()->light(),
                            scene, est.it().position());

                        real weight = pLe / (pLi + pLe);
                        le = le * weight;
                    }

                    // Li is weighted by MIS if the light is intersectable
                    // by tha pathtracer, that is an area light.
                    if(nee.light() && 
                        nee.light()->type() == Light::LightType::INTERSECTABLE){

                        // Probability of intersecting it as an area
                        // light in nee.
                        real pLi = nee.pdf();
                        // Probability of intersecting instance
                        // using pathtracing (in solid angles).
                        Vector3 wi = (nee.position() - sp.position()).normalized();
                        real pLe = MonteCarlo::evaluateSolidAnglePdf(wo, sp, wi);
                        real weight = pLi / (pLi + pLe);
                        li = li * weight;
                    }

                    // ------ Then we update the throughput, color,
                    // ray, and surface point for the next bounce ------


                    // Finally, we add li and le to the color
                    color = color + le + li;
                    // And we update the throughput
                    throughput = throughput * est.weight();
                    // The new ray starts at the last intersected point
                    // and points towards the new intersected point.
                    currRay = Ray(sp.position(), est.wi());
                    // And the new intersected point is the one we sampled.
                    sp = est.it();
                }

                averageLe = (averageLe * k + color) * (1.0 / (k+1.0));

            }

            return averageLe;
        }

    };

}

#endif