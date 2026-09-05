
#ifndef PATH_TRACER_MONTE_CARLO_HPP
#define PATH_TRACER_MONTE_CARLO_HPP

#include "monteCarloEstimator.hpp"
#include "../core/sceneUtil.hpp"
#include "../intersection/areaSample.hpp"

namespace pathtracer{

    class MonteCarlo{

    
    public:

        // Calculates the estimator for a path tracer using solid angles.
        // We have the current point at it, and the outgoing direction wo.
        // We sample an incoming direction wi, get the radiance from wi,
        // calculate the weight (bsdf, cosine, pdf...) at the current
        // point it, which we can use to multiply the incoming
        // light emission (from the newly sampled point, not it).
        static MonteCarloEstimator sampleSolidAngle(const Vector3& wo, 
            const SurfacePoint& it, const Scene& scene){

            // We will use the BSDF at the point of contact x
            // to get the direction wi, from where the light comes,
            // as the BSDF is the only object that knows what directions
            // are valid based on the material type.
            BsdfSample sample = it.sampleBsdf(wo);

            Vector3 weight = sample.weight();

            // First we get the radiance incoming from wi
            // (With small offset to avoid self intersection)
            Ray ray(it.position() + sample.wi() * EPSILON, sample.wi());
            Intersection newIt = scene.intersect(ray);

            // If we don't intersect any object
            if(!newIt){
                // Then we can return a sample with no visibility
                // to indicate this shoots into the void.
                return MonteCarloEstimator(Vector3::ORIGIN, weight, sample.wi(),
                    Intersection::NO_HIT, sample.pdf(), 0, false);
            }

            Vector3 radiance = newIt.evaluateEmission(-sample.wi());
            
            real distance = newIt.t();
            

            // Because this is a solid angle estimator, the
            // sampled point is always visible.
            return MonteCarloEstimator(radiance, weight, sample.wi(),
                newIt, sample.pdf(), distance, true);
        }

        
        // Samples a point on a light, for NEE (area formulation).
        static MonteCarloLightSample sampleLight(const Vector3& wo, 
            const SurfacePoint& it, const Scene& scene){

            if(scene.lightCount() <= 0){
                return MonteCarloLightSample();
            }

            // First we pick a random light with uniform probability.
            const Light* light = UniformLight::sample(scene);

            // The pdf of choosing this instance
            real pdfInstance = UniformLight::pdf(scene, light);

            // Then we sample the light for a point and radiance,
            // which returns a light sample in world coordinates
            // in solid angle measure.
            LightSample s = light->sample(it.position());

            if(!s.isValid()) {
                return MonteCarloLightSample();
            }
            
            // The pdf of choosing this point is the pdf of
            // choosing the light times the pdf of choosing the point
            // on the light.
            real pdfPoint = pdfInstance * s.pdf();

            // Note that we evaluate, not sample the bsdf, since
            // we already have a wi.
            // We also disregard everything returned by the evaluate function
            // except the bsdf, as we already have the pdf we need
            // (the pdf returned is that of the bsdf sampling the given wi).
            BsdfSample bsdfEval = it.evaluateBsdf(wo, s.wi());
            
            if(bsdfEval.cosine() <= 0) {
                return MonteCarloLightSample();
            }
            
            // The visibility term.
            // Note that this works for directional lights
            // as they generate a very far away point.
            bool visibility = scene.visibility(it.position(), s.position());

            Vector3 weight = bsdfEval.bsdf() * bsdfEval.cosine() * (1.0 / pdfPoint);

            return MonteCarloLightSample(s.radiance(), weight, 
                s.position(), light, pdfPoint, visibility);
        }


        // We already have the functions that return the monte
        // carlo estimators after sampling a direction, or point,
        // on the instances or lights.
        
        // Now we need the reverse query. That is, given a direction,
        // or point, what is the probability density that one of these
        // schemes would have sampled it.
        // We need this for MIS.


        // Returns the pdf of having sampled a particular
        // direction wi at a specific surface point.
        static real evaluateSolidAnglePdf(const Vector3& wo, 
            const SurfacePoint& it, const Vector3& wi){

            // We evaluate the bsdf with the given wi, which returns
            // the pdf of sampling said wi according to the bsdf's
            // sampling strategy.
            BsdfSample sample = it.evaluateBsdf(wo, wi);

            return sample.pdf();
        }


        // Returns the pdf of having sampled a particular
        // point on a particular instance using the light sampling.
        // PDF is returned in solid angles.
        static real evaluateLightPdf(const Vector3& origin, 
            const Light* light, const Scene& scene,
            const Vector3& point){

            if(scene.lightCount() <= 0){
                return 0.0;
            }

            if(!light->isIntersectable()){
                return 0.0;
            }

            // The pdf of choosing this instance
            int lightCount = scene.lightCount();
            real pdfInstance = 1.0 / lightCount;

            // Instead of sampling the area, we evaluate the area
            // sample assuming we had gotten this particular point.
            LightSample sample = light->evaluateLightSample(origin, point);

            return sample.pdf() * pdfInstance;
        }

    };

}

#endif