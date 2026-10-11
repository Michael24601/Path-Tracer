
#include "integrator/pathTracerGuided.hpp"
#include "bsdf/bsdfSample.hpp"
#include "core/instance.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "intersection/intersection.hpp"
#include "math/mathUtil.hpp"
#include "bsdf/bsdf.hpp"
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTree.hpp"
#include "math/constants.hpp"
#include "logger.hpp"

namespace pathtracer{
    

    PathTracerGuided::PathTracerGuided(int maxDepth) :
        m_maxDepth{maxDepth} {}


    Vector3 PathTracerGuided::color(
        const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        // Used for path guiding training
        std::vector<SurfacePoint> positions;
        std::vector<Vector3> weights;
        std::vector<Vector3> directions;
        std::vector<real> pdfs;

        Vector3 finalEmission(0.0);

        // This includes the emission at every single point,
        // which is the same as 

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

                finalEmission = emission;
                
                break;
            }

            // ------ This next step is normal pathtracer ------

            // Russian roulette
            real p = Util::russianRoulette(throughput, i);
            if (Random::next() > p){
                break;
            }

            // If we have a non-null guide tree, we can use it to guide
            // samples.

            // ---------------------- PATH GUIDING ------------------------

            // This is sampled using the guided tree (if it exists),
            // with fallbacks to bsdf handled already.
            GuidedSample sample = sampleGuided(wo, it, scene);

            if(!sample.isValid){
                break;
            }

            // And we update the throughput (along with RR probability)
            sample.weight = sample.weight * (1.0f / p);

            // We then store, in a stack, the position, and the throughput
            // at this point.
            if(m_trainTree){
                weights.push_back(sample.weight);
                positions.push_back(it);
                directions.push_back(sample.wi);
                pdfs.push_back(sample.pdf);
            }

            throughput = throughput * sample.weight;
            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + sample.wi * SHADOW_EPSILON, sample.wi);
        }

        // Contributions at each vertex are 0.0 since no NEE
        std::vector<Vector3> contributions(positions.size(), Vector3(0.0));

        // Saved for training
        recordPath(positions, directions, weights, pdfs, contributions, 
            finalEmission, scene);

        return color;
    }

}