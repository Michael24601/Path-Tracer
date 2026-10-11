
#include "integrator/pathTracerNeeGuided.hpp"
#include <cmath>
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
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTree.hpp"
#include "math/constants.hpp"


namespace pathtracer{


    PathTracerNeeGuided::PathTracerNeeGuided(int maxDepth) :
        m_maxDepth{maxDepth}{}


    Vector3 PathTracerNeeGuided::color(
        const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        // If the last frame is specular, then no NEE was performed,
        // so we can instead count light bsdf sample contribution
        // if it was hit.
        bool isLastFrameSpecular = false;

        std::vector<Vector3> positions;
        std::vector<Vector3> weights;
        std::vector<Vector3> directions;
        std::vector<real> pdfs;

        // The contributions at each path point (not including last one),
        // from next event estimation.
        std::vector<Vector3> contributions;
        Vector3 finalEmission(0.0);

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

                    finalEmission = emission;

                    break;
                }
                else{
                    // If not first bounce, ignore, since using NEE
                    // (no double count).
                    break;
                }
            }

            // ------------- This next step is NEE -------------


            Vector3 contrib(0.0);

            // We only perform NEE if the current surface is not
            // delta surface (specular).
            if(it.instance()->bsdf()->isSpecular()){
                isLastFrameSpecular = true;
            }
            else{
                isLastFrameSpecular = false;

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
                bool visibility =
                    scene.visibility(it.position(), s.wi(), s.distance());

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

                    if(!bsdfEval.isInvalid() && bsdfEval.cosine() > 0) {

                        Vector3 neeWeight = bsdfEval.bsdf() * bsdfEval.cosine() * (1.0 / pdfPoint);
                        Vector3 emission = s.radiance();
                        
                        contrib = emission * neeWeight;
                        color = color + contrib * throughput;
                    }
                }
            }
            
            contributions.push_back(contrib);

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

            sample.weight = sample.weight * (1.0f / p);

            // We then store, in a stack, the position, and the throughput
            // at this point.
            if(m_trainTree){
                weights.push_back(sample.weight);
                positions.push_back(it.position());
                directions.push_back(sample.wi);
                pdfs.push_back(sample.pdf);
            }

            // And we update the throughput (along with RR probability)
            throughput = throughput * sample.weight;

            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + sample.wi * SHADOW_EPSILON, sample.wi);
        }   

        if(m_trainTree){
            recordPath(positions, directions, weights, pdfs, contributions, 
                finalEmission, scene);
        }

        return color;
    }

}