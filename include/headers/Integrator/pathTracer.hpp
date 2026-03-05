
#ifndef PATH_TRACER_PATH_TRACER_HPP
#define PATH_TRACER_PATH_TRACER_HPP

#include "integrator.hpp"
#include "monteCarlo.hpp"

namespace pathtracer{

    class PathTracer: public Integrator{

    private:

        int m_depth;
        
    public:

        virtual Vector3 color(const Ray& ray, const Scene& scene) const 
            override{

            Ray currRay = ray;
            Vector3 le = Vector3(0.0);
            Vector3 throughput = Vector3(1.0);
            
            for(int i = 0; i < m_depth; i++){

                Intersection it = scene.intersect(currRay);

                if(!it){
                    break;
                }
            
                MonteCarloEstimator est = 
                    MonteCarlo::sampleSolidAngle(currRay.direction(), it);

                throughput = throughput * est.weight();
                le = le + (est.radiance() * throughput);
                
                currRay = Ray(it.position(), est.wi());
            }

        };

        
    };

}

#endif