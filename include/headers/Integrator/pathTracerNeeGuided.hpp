 
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

        Vector3 color(const Ray& ray, const Scene& scene) override{

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

            for(int i = 0; i < m_maxDepth; i++){

                Vector3 wo = -currRay.direction();

                // First we intersect the scene
                Intersection it = scene.intersect(currRay);
                // If no hits (we can break or sample envmap)
                if(!it){
                    recordPath(positions, directions, weights, Vector3(0.0), scene);
                    break;
                }

                // If we have an emissive surface, we only consider
                // it on the first bounce, since we are using
                // NEE for every other bounce
                if(it.instance()->emission()){

                    if(i == 0 || isLastFrameSpecular){
                        Vector3 emission = it.evaluateEmission(wo);
                        color = color + emission * throughput;

                        if(m_trainTree){
                            recordPath(positions, directions, weights, emission, scene);
                        }

                        break;
                    }
                    else{
                        // If not first bounce, ignore, since using NEE
                        // (no double count).
                        break;
                    }
                }


                // ------------- This next step is NEE -------------


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
                            Vector3 emission = s.radiance();
                            color = color + emission * throughput * neeWeight;

                            if(m_trainTree){

                                // We add these to complete the path
                                positions.push_back(it.position());
                                directions.push_back(s.wi());
                                weights.push_back(neeWeight);

                                recordPath(positions, directions, weights, emission, scene);

                                // We then remove them as they are not part
                                // of the path going forward.
                                positions.pop_back();
                                weights.pop_back();
                                directions.pop_back();
                            }
                        }
                    }
                }

                // ------ This next step is normal pathtracer ------

                // Russian roulette
                real p = Util::russianRoulette(throughput);
                if (Random::next() > p){
                    recordPath(positions, directions, weights, Vector3(0.0), scene);
                    break;
                }


                // If we have a non-null guide tree, we can use it to guide
                // samples.

                // ---------------------- PATH GUIDING ------------------------

                Vector3 wi;
                Vector3 weight;
                real pdf;               
                BsdfSample sample;

                if(m_guideTree && !it.instance()->bsdf()->isSpecular()){
                
                    Vector3 localPoint = toTreeLocalSpace(it.position(), scene);
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
                
               
                if(!m_guideTree || it.instance()->bsdf()->isSpecular() || sample.isInvalid()){ 
                    sample = it.sampleBsdf(wo);
                    wi = sample.wi();
                    weight = sample.weight();
                    pdf = sample.pdf();
                }

                // RR probability
                weight = weight * (1.0f / p);

                // We then store, in a stack, the position, and the throughput 
                // at this point.
                if(m_trainTree){
                    weights.push_back(weight);
                    positions.push_back(it.position());
                    directions.push_back(wi);
                }

                // And we update the throughput (along with RR probability)
                throughput = throughput * weight;
                // The new ray starts at the last intersected point
                // and points towards the new intersected point.
                currRay = Ray(it.position() + wi * SHADOW_EPSILON, wi);   
                
            }
            
                
            return color;
        }

    };

}

#endif