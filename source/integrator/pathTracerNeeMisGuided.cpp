
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


        std::vector<Vector3> positions;
        std::vector<Vector3> weights;
        std::vector<Vector3> directions;
        std::vector<real> pdfs;

        // The contributions at each path point (not including last one),
        // from next event estimation.
        std::vector<Vector3> contributions;
        Vector3 finalEmission(0.0);

        // If the last frame is specular, then no NEE was performed,
        // which is relevant information for MIS.
        BsdfSample lastFrameSample(BsdfSample::INVALID);
        bool isLastFrameSpecular = false;

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
                    !lastFrameSample.isDelta() && scene.lightCount() > 0){

                    // The pdf of the bsdf having generated it is:
                    real pdf_bsdf = lastFrameSample.pdf();

                    // The pdf of the NEE is computed and converted to solid
                    // angles.
                    real distance = it.t();
                    Vector3 direction = wo;
                    SurfaceSample s{ it.position(), it.triangleIndex()};
                    AreaSample areaSample = it.instance()->evaluateAreaSample(s);

                    real pdf_nee = areaSample.pdf();

                    // This is the cosine term on the light
                    real cos = std::max(it.shadingNormal().dot(direction), 0.0);

                    pdf_nee *= (distance * distance) / cos;

                    // We also multiply p_nee by probability of choosing
                    // said light
                    pdf_nee *= 1.0 / scene.lightCount();

                    if(pdf_bsdf > 0.0 && pdf_nee > 0.0 && cos > 0.0) {

                        misWeight = pdf_bsdf / (pdf_nee + pdf_bsdf);
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

                if(!bsdfEval.isInvalid() && bsdfEval.cosine() > 0 && s.visibility) {

                    //---------------------- MIS -----------------------

                    real misWeight = 1.0;

                    // If the light is intersectable, then bsdf can
                    // sample it.
                    if(s.sample.caster()->isIntersectable()){

                        // This is the current pdf that samples the
                        // light. It needs to be in solid angles,
                        // and the function returns it in solid angles.
                        real pdf_nee = s.pdf;

                        // The pdf that the bsdf would have sampled it
                        // is inside the evaluated sample.
                        real pdf_bsdf = bsdfEval.pdf();

                        if(pdf_bsdf > 0.0 && pdf_nee > 0.0) {

                            misWeight = pdf_nee / (pdf_nee + pdf_bsdf);
                        }
                    }

                    // Contribution includes pdf already
                    contrib = s.contribution * misWeight;

                    // Contribution includes pdf
                    color = color + (contrib * throughput * bsdfEval.bsdf() 
                        * bsdfEval.cosine());

                    if(m_trainTree){
                        recordVertex(it.position(), s.sample.wi(), contrib, scene);
                    }
                }
            }


            contributions.push_back(contrib);


            // ------------------ This next step is guiding --------------------

            // Russian roulette
            real p = Util::russianRoulette(throughput);

            if (Random::next() > p){
                break;
            }


            Vector3 wi;
            Vector3 weight;
            real pdf;
            BsdfSample sample = BsdfSample::INVALID;

            if(m_guideTree && !it.instance()->bsdf()->isSpecular()){

                Vector3 localPoint =
                    toTreeLocalSpace(it.position(), scene);

                auto dTree = m_guideTree->getDTree(localPoint);

                // We use MIS to blend the two guiding techniques
                if(Random::next() <= m_alpha){
                    wi = dTree->sample();
                    sample = it.evaluateBsdf(wo, wi);
                }
                else{
                    sample = it.sampleBsdf(wo);
                    wi = sample.wi();
                }

                // Whether wi we generated by either method, these
                // values record the pdf that the either method generated
                // wi. We can assume the pdf is always valid since
                // we only do this for non-specular surfaces with valid pdf.
                real guidePdf = dTree->pdf(wi);
                real bsdfPdf = sample.pdf();

                pdf = m_alpha * guidePdf + (1.0 - m_alpha) * bsdfPdf;
                weight = sample.bsdf() * sample.cosine() / pdf;
            }
            else {
                sample = it.sampleBsdf(wo);
                wi = sample.wi();
                weight = sample.weight();
                pdf = sample.pdf();
            }

            if(sample.isInvalid()){
                break;
            }

            weight = weight * (1.0f / p);

            // We then store, in a stack, the position, and the throughput
            // at this point.
            if(m_trainTree){
                weights.push_back(weight);
                positions.push_back(it.position());
                directions.push_back(wi);
                pdfs.push_back(pdf);
            }

            // And we update the throughput (along with RR probability)
            throughput = throughput * weight;

            // The new ray starts at the last intersected point
            // and points towards the new intersected point.
            currRay = Ray(it.position() + wi * SHADOW_EPSILON, wi);

            lastFrameSample = sample;
        }


        if(m_trainTree){
            recordPath(positions, directions, weights, pdfs, contributions, 
                finalEmission, scene);
        }

        return color;
    }

}

