
#include "integrator/PathTracerNeeMisGuided.hpp"
#include <algorithm>
#include "bsdf/bsdfSample.hpp"
#include "core/instance.hpp"
#include "core/random.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "core/sceneUtil.hpp"
#include "intersection/areaSample.hpp"
#include "intersection/intersection.hpp"
#include "light/light.hpp"
#include "light/lightSample.hpp"
#include "math/mathUtil.hpp"
#include "shapes/shape.hpp"
#include "bsdf/bsdf.hpp"
#include "math/constants.hpp"
#include "nextEventEstimation/nextEventEstimation.hpp"
#include "pathGuiding/kdTree.hpp"
#include "pathGuiding/quadTree.hpp"
#include "math/constants.hpp"
#include "logger.hpp"


namespace pathtracer{

    PathTracerNeeMisGuided::PathTracerNeeMisGuided(int maxDepth) :
        m_maxDepth{maxDepth} {}


    Vector3 PathTracerNeeMisGuided::color(
        const Ray& ray, const Scene& scene) {

        Ray currRay = ray;
        Vector3 color = Vector3(0.0);
        Vector3 throughput = Vector3(1.0);

        std::vector<SurfacePoint> positions;
        std::vector<Vector3> weights;
        std::vector<Vector3> directions;
        std::vector<real> pdfs;

        // The contributions at each path point (not including last one),
        // from next event estimation.
        std::vector<Vector3> contributions;
        Vector3 finalEmission(0.0);

        // If the last frame is specular, then no NEE was performed,
        // which is relevant information for MIS.
        bool isLastFrameSpecular = false;
        GuidedSample lastFrameSample;
        Vector3 previousPos(0.0);

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

                //------------------------- MIS ----------------------------

                real misWeight = 1.0;

                // If the emissive surface is a light (NEE sees it)
                // (If delta we can ignore MIS since NEE can't sample it,
                // and there isn't really a numerical pdf).
                if(i > 0 && it.instance()->light() &&
                    !isLastFrameSpecular && scene.lightCount() > 0){

                    // The pdf of the bsdf having generated it is:
                    real pdf_bsdf = lastFrameSample.pdf;

                    // The pdf of the NEE is computed in solid angles by
                    // calling NEE evaluate instead of sample, and passing
                    // the hit point (assuming it is emissive and a light).
                    NextEventSample nee = 
                        NextEventEstimation::evaluate(scene, previousPos, it);

                    if(pdf_bsdf > 0.0 && nee.sample.isValid() && nee.pdf > 0.0) {
                        misWeight = pdf_bsdf / (nee.pdf + pdf_bsdf);
                    }
                }

                Vector3 emission = it.evaluateEmission(wo);
                emission = emission * misWeight;
                color = color + emission * throughput;

                finalEmission = emission;

                break;
            }

            // ------------- This next step is NEE -------------


            Vector3 contrib(0.0);


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

                if(!bsdfEval.isInvalid() && bsdfEval.cosine() > 0 && s.visibility
                    && s.sample.isValid()) {

                    //---------------------- MIS -----------------------

                    real misWeight = 1.0;

                    // If the light is intersectable, then bsdf can
                    // sample it.
                    if(s.sample.caster()->isIntersectable()){

                        // This is the current pdf that samples the
                        // light. It needs to be in solid angles,
                        // and the function returns it in solid angles.
                        real pdf_nee = s.pdf;

                        // The pdf that the other strategy, this time
                        // the guided one, could have sampled wi.
                        GuidedSample other = evaluateGuided(wo, s.sample.wi(), it, scene);

                        if(pdf_nee > 0.0 && other.isValid && other.pdf > 0.0) {

                            misWeight = pdf_nee / (pdf_nee + other.pdf);
                        }
                    }

                    // Contribution includes pdf already
                    contrib = s.contribution * misWeight;

                    // We train this vertex only using the incoming radiance / pdf
                    if(m_trainTree){
                        recordVertex(it, s.sample.wi(), contrib, scene);
                    }

                    // However, when we train the full path later, the added
                    // contribution must also include the bsdf and cosine
                    contrib = contrib * bsdfEval.bsdf() * bsdfEval.cosine();

                    // Contribution includes pdf
                    color = color + (contrib * throughput);
                }
            }


            contributions.push_back(contrib);


            // ------------------ This next step is guiding --------------------

            // Russian roulette
            real p = Util::russianRoulette(throughput, i);

            if (Random::next() > p){
                break;
            }

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
                positions.push_back(it);
                directions.push_back(sample.wi);
                pdfs.push_back(sample.pdf);
            }

            // And we update the throughput (along with RR probability)
            throughput = throughput * sample.weight;

            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + sample.wi * SHADOW_EPSILON, sample.wi);

            lastFrameSample = sample;
            previousPos = it.position();
        }


        if(m_trainTree){
            recordPath(positions, directions, weights, pdfs, contributions, 
                finalEmission, scene);
        }

        return color;
    }

}

