
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

            Vector3 weight = sample.bsdf() * sample.cosine()
                * (1.0 / sample.pdf());

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


        // Samples a point using the area formulation for a regular
        // path tracer (not NEE).
        static MonteCarloEstimator sampleArea(const Vector3& wo, 
            const SurfacePoint& it, const Scene& scene){

            // First we pick a random object with uniform probability.
            const Instance* inst = UniformInstance::sample(scene);

            // The pdf of choosing this instance
            real pdfInstance = UniformInstance::pdf(scene, inst);

            // Instead of using the BSDF to sample a direction,
            // we just randomly pick a point on any surface in the
            // scene.

            AreaSample sample = inst->sampleArea();
            Vector3 posWorld = sample.position();
            Vector3 normalWorld = sample.shadingNormal();

            MonteCarloEstimator estimate;

            // Calculates wi based on the sampling. Unlike solid
            // angles, we don't sample wi according to the BSDF.
            Vector3 wi = posWorld - it.position();
            real dist = wi.length();
            wi = wi * (1.0 / dist);

            // The emission comes from the newly sampled point,
            // not the old intersected point.
            Vector3 radiance = sample.evaluateEmission(wo);

            // Because wi is sampled using the area, we evaluate the
            // BSDF instead of sampling it, with the existing wi.

            // NOTE: We only care about the bsdf() from bsdfEval,
            // the pdf is to be disregarded since we already have
            // the pdf of choosing the point we sampled.
            // The pdf returned by bsdfEval is the pdf of sampling
            // wi according to the bsdf sample funciton, which we don't
            // want for the area formulation.
            BsdfSample bsdfEval = it.evaluateBsdf(wo, wi);

            // This means deterministic BSDFs return an invalid,
            // as they require a specific wi to work, and a random
            // one will have a probability of 0 of working.
            if(bsdfEval.isInvalid()){
                return estimate;
            }

            // The visibility term
            bool visibility = scene.visibility(it.position(), posWorld);

            // The pdf is the pdf of choosing this instance, times
            // the area pdf in world coordinates.
            real pdfPoint = pdfInstance * sample.pdf();

            // We have two choices:
            // We either calculate the area formulation estimator,
            // which includes a geometry term cosX * cosY / dist^2,
            // or we can calculate the estimator in the solid
            // angle formulation, where we only have cosX,
            // but the pdf has to be transformed to the solid
            // angle pdf by multiplying by dist^2/cosY.
            // The result is the exact same, since we eventually
            // divide by the pdf.
            // We choose the latter, so that we have a common ground
            // pdf we can use for MIS that is always in solid angles.
            real cosineX = std::max(0.0, it.shadingNormal().dot(wi));
            real cosineY = std::max(0.0, normalWorld.dot(-wi));
            real pdfSolidAngle = pdfPoint * (dist * dist) / cosineY;

            Vector3 weight = bsdfEval.bsdf() * cosineX
                * (1.0 / pdfSolidAngle);

            real distance = (posWorld - it.position()).length();

            return MonteCarloEstimator(radiance, weight, wi, sample, 
                pdfSolidAngle, distance, visibility);
        }

        
        // Samples a point on a light, for NEE.
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
        // point on a particular instance using the area sampling.
        static real evaluateAreaPdf(const Instance* inst, const Scene& scene,
            const Vector3& point){

            // The pdf of choosing this instance
            int instanceCount = scene.instanceCount();
            real pdfInstance = 1.0 / instanceCount;

            // Instead of sampling the area, we evaluate the area
            // sample assuming we had gotten this particular point,
            // which returns to us the pdf of having chosen said
            // point.
            AreaSample sample = inst->evaluateAreaSample(point);

            return sample.pdf() * pdfInstance;
        }


        // Returns the pdf of having sampled a particular
        // point on a particular instance using the light sampling.
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