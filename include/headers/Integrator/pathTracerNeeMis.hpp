 
#ifndef PATH_TRACER_PATH_TRACER_NEE_MIS_HPP
#define PATH_TRACER_PATH_TRACER_NEE_MIS_HPP

#include "integrator.hpp"
#include "../core/sceneUtil.hpp"

namespace pathtracer{

    // A pure path tracer
    class PathTracerNeeMis: public Integrator{

    private:

        int m_maxDepth;
        
    public:


        // Uses both NEE and BSDF sampling, but instead of using one
        // or the other, blends them with MIS.
        PathTracerNeeMis(int maxDepth) : m_maxDepth{maxDepth}{}

        Vector3 color(const Ray& ray, const Scene& scene) const override{

            Ray currRay = ray;
            Vector3 color = Vector3(0.0);
            Vector3 throughput = Vector3(1.0);

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
                    if(i > 0 && it.instance()->light() && !lastFrameSample.isDelta()
                        && scene.lightCount() > 0){
                        
                        // The pdf of the bsdf having generated it is:
                        real pdf_bsdf = lastFrameSample.pdf();
                        
                        // The pdf of the NEE is computed and converted to solid
                        // angles.
                        real distance = it.t();
                        Vector3 direction = wo;
                        SurfaceSample s{it.position(), it.triangleIndex()};
                        AreaSample areaSample = it.instance()->evaluateAreaSample(s);
                        real pdf_nee = areaSample.pdf();
                        // This is the cosine term on the light
                        float cos = std::max(it.shadingNormal().dot(direction), 0.0);
                        pdf_nee *= (distance * distance) / cos;
                        // We also multiply p_nee by probability of choosing said light
                        pdf_nee *= 1.0 / scene.lightCount();

                        if (pdf_bsdf > 0.0 && pdf_nee > 0.0 && cos > 0.0) {
                            misWeight = pdf_bsdf / (pdf_nee + pdf_bsdf);
                        }

                    }

                    Vector3 emission = it.evaluateEmission(wo);
                    color = color + emission * throughput * misWeight;
                    break;
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
                        // on the light (in solid angles already).
                        real pdfPoint = pdfInstance * s.pdf();

                        // Note that we evaluate, not sample the bsdf, since
                        // we already have a wi, so we just need the value
                        // and cosine. Also we ignore the pdf it returns since
                        // this is the pdf of the bsdf having generated said
                        // path (used in MIS for example).
                        BsdfSample bsdfEval = it.evaluateBsdf(wo, s.wi());

                        if(!bsdfEval.isInvalid() && bsdfEval.cosine() > 0) {

                            //---------------------- MIS -----------------------

                            real misWeight = 1.0;

                            // If the light is intersectable, then bsdf can
                            // sample it.
                            if(light->isIntersectable()){

                                // This is the current pdf that samples the
                                // light. It needs to be in solid angles,
                                // and the function returns it in solid angles.
                                real pdf_nee = pdfPoint;

                                // The pdf that the bsdf would have sampled it
                                // is inside the evaluated sample.
                                real pdf_bsdf = bsdfEval.pdf();

                                if (pdf_bsdf > 0.0 && pdf_nee > 0.0) {
                                    misWeight = pdf_nee / (pdf_nee + pdf_bsdf);
                                }
                            }

                            Vector3 neeWeight = bsdfEval.bsdf() * bsdfEval.cosine() * (1.0 / pdfPoint);
                            color = color + s.radiance() * throughput * neeWeight * misWeight;
                        }
                    }
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
                currRay = Ray(it.position() + sample.wi() * SHADOW_EPSILON, sample.wi());  
                
                lastFrameSample = sample;
            }

            return color;
        }

    };

}

#endif