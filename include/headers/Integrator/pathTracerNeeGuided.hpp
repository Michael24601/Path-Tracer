 
#ifndef PATH_TRACER_PATH_TRACER_NEE_GUIDED_HPP
#define PATH_TRACER_PATH_TRACER_NEE_GUIDED_HPP

#include "integrator.hpp"
#include "guidable.hpp"
#include "../core/sceneUtil.hpp"
#include <cmath>

namespace pathtracer{

    // A pure path tracer
    class PathTracerNeeGuided: public Integrator, public Guidable{

    private:

        int m_maxDepth;

        // Weight used when combining bsdf sampling with guiding
        // (defensive sampling).
        real m_alpha;
        
        
    public:

        PathTracerNeeGuided(int maxDepth, real alpha) : m_maxDepth{maxDepth},
            m_alpha{alpha}{}

        Vector3 color(const Ray& ray, const Scene& scene) const override{

            Ray currRay = ray;
            Vector3 color = Vector3(0.0);
            Vector3 throughput = Vector3(1.0);

            // If the last frame is specular, then no NEE was performed,
            // so we can instead count light bsdf sample contribution
            // if it was hit.
            bool isLastFrameSpecular = false;

            std::stack<Vector3> positionStack;
            std::stack<Vector3> weightStack;
            std::stack<Vector3> directionStack;
            std::stack<Vector3> emissionStack;

            // The emission that comes from bsdf sampling
            Vector3 bsdfEmission(0.0);

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
                    
                        bsdfEmission = emission;
                        break;
                    }
                    else{
                        // If not first bounce, ignore, since using NEE
                        // (no double count).
                        break;
                    }
                }


                // ------------- This next step is NEE -------------

                // If none is added, we add 0
                bool addedEmission = false;

                // We only perform NEE if the current surface is not
                //  delta surface (specular)
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
                        if(!bsdfEval.isInvalid() && bsdfEval.cosine() > 0) {
                            Vector3 neeWeight = bsdfEval.bsdf() * bsdfEval.cosine() * (1.0 / pdfPoint);
                            Vector3 emission = s.radiance() * neeWeight;
                            color = color + emission * throughput;

                            // Note that we push the emission into the emission stack
                            // for guiding with the nee weight already in it,
                            // since it is only ever reachable through that weight.
                            emissionStack.push(emission);
                            addedEmission = true;
                        }
                    }
                }

                if(!addedEmission){
                    emissionStack.push(Vector3(0.0));
                }


                // ------ This next step is normal pathtracer ------

                // Russian roulette
                // No RR when not training (final render)
                real p = 1.0;
                if(m_trainTree){
                    p = Util::russianRoulette(throughput);
                    if (Random::next() > p){
                        break;
                    }
                }


                // If we have a non-null guide tree, we can use it to guide
                // samples.

                // ---------------------- PATH GUIDING ------------------------

                Vector3 wi;
                Vector3 localWi;
                Vector3 weight;
                real pdf;               
                BsdfSample sample;

                if(m_guideTree && !it.instance()->bsdf()->isSpecular()){
                
                    Vector3 localPoint = toTreeLocalSpace(it.position(), scene);
                    auto dTree = m_guideTree->getDTree(localPoint);

                    // We use MIS to blend the two guiding techniques
                    if(Random::next() < m_alpha){
                        // Wi is in local coordinates
                        localWi = dTree->sample();
                        wi = it.shadingFrame().inverseTransformDirection(localWi);
                        sample = it.evaluateBsdf(wo, wi);
                    }
                    else{
                        sample = it.sampleBsdf(wo);
                        wi = sample.wi();
                        localWi = it.shadingFrame().transformDirection(wi);
                    }
                    
                    // Whether wi we generated by either method, these
                    // values record the pdf that the either method generated
                    // wi. We can assume the pdf is always valid since
                    // we only do this for non-specular surfaces with valid pdf.
                    real guidePdf = dTree->pdf(localWi);
                    real bsdfPdf = sample.pdf();
                    
                    pdf = m_alpha * guidePdf + (1 - m_alpha) * bsdfPdf;
                    weight = sample.bsdf() * sample.cosine() / pdf;
                }
                else{
                    sample = it.sampleBsdf(wo);
                    wi = sample.wi();
                    weight = sample.weight();
                    localWi = it.shadingFrame().transformDirection(wi);
                    pdf = sample.pdf();
                }

                // RR probability
                weight = weight * (1.0f / p);

                // We then store, in a stack, the position, and the throughput 
                // at this point.
                if(m_trainTree){
                    weightStack.push(weight);
                    positionStack.push(it.position());
                    directionStack.push(localWi);
                }

                // And we update the throughput (along with RR probability)
                throughput = throughput * weight;
                // The new ray starts at the last intersected point
                // and points towards the new intersected point.
                currRay = Ray(it.position() + wi * SHADOW_EPSILON, wi);   
                
            }

            // Now, at the end, we train the training tree, if there is one
            if(m_trainTree){

                Vector3 intensity = bsdfEmission;

                while(!positionStack.empty()){

                    Vector3 localPoint = toTreeLocalSpace(positionStack.top(), scene);
                    Vector3 direction = directionStack.top();

                    intensity = intensity * weightStack.top();

                    if(isfinite(intensity.x()) &&
                        isfinite(intensity.y()) &&
                        isfinite(intensity.z())){
                        m_trainTree->accumulate(
                            localPoint,
                            direction,
                            intensity.luminance()
                        );
                    }
                    else{
                        // Weight is corrupted from somewhere
                        break;
                    }

                    // We also add NEE emission
                    // Note that the tree is meant to allow us to choose
                    // good directions. Since the NEE is based on current
                    // position and not the current sampled direction wi,
                    // it won't contribute to this vertex's accumulation,
                    // but does to the all teh rpevious ones, so we add it
                    // after accumulating.
                    intensity = intensity + emissionStack.top(); 

                    // We then pop the stack etc...
                    weightStack.pop();
                    positionStack.pop();
                    directionStack.pop();
                    emissionStack.pop();
                }

            }
            
                
            return color;
        }

    };

}

#endif