
#ifndef PATH_TRACER_MONTE_CARLO_HPP
#define PATH_TRACER_MONTE_CARLO_HPP

#include "estimator.hpp"
#include "../intersection/areaSample.hpp"

namespace pathtracer{

    class MonteCarlo{

    private:
    
    
        // Calculates the estimator for a path tracer using the area
        // formulation.
        // It takes as input the object we have decided to sample,
        // as well as the pdf of choosing said object.
        // Can be used both for regular path tracing and for
        // next event estimation.
        static AreaEstimator sampleArea(const Vector3& wo, 
            const SurfacePoint& it, const Instance* const inst, 
            real pdfInstance, const Scene& scene){

            // Instead of using the BSDF to sample a direction,
            // we just randomly pick a point on any surface in the
            // scene.

            // Both the position and the pdf are in the shape's
            // local coordinates, so we can transform them.
            AreaSample sample = inst->shape()->sampleSurfaceArea();

            Vector3 posWorld = 
                inst->transform().transform(sample.position());

            // The transform matrix is used directly since we don't
            // want it normalized yet.
            Transform m = inst->transform().inverseTranspose();
            Vector3 normalWorld = m.transform(sample.shadingNormal());

            // Transforming a local pdf to a world pdf
            real worldPdf = sample.pdf() / (normalWorld.length() 
                * std::abs(inst->transform().determinant()));

            // The total pdf is just the pdf of choosing the point
            // multilplied by the pdf of choosing the shape.
            real pdfPoint = worldPdf * pdfInstance;

            // Calculates wi based on the sampling. Unlike solid
            // angles, we don't sample wi according to the BSDF.
            Vector3 wi = posWorld - it.position();
            real dist = wi.length();
            wi = wi * (1.0 / dist);
            Vector3 radiance = it.evaluateEmission(wo);

            // Because wi is sampled using the area, we evaluate the
            // BSDF instead of sampling it, with the existing wi.
            BsdfEvaluation bsdfEval = it.evaluateBsdf(wo, wi);

            // This means deterministic BSDFs return an invalid,
            // as they require a specific wi to work, and a random
            // one will have a probability of 0 of working.
            if(bsdfEval.isInvalid()){
                return AreaEstimator(radiance, Vector3(0.0), sample, 
                    0.0, false);
            }

            // Otherwise we add the geometry term and we proceed.
            real cosineX = std::abs(it.shadingNormal().dot(wi));
            real cosineY = std::abs(normalWorld.dot(-wi));
            real geometryTerm = (cosineX * cosineY) / (dist * dist);

            // The visibility term
            bool visibility = scene.visibility(it.position(), posWorld, wi);

            Vector3 weight = bsdfEval.bsdf() * geometryTerm 
                * (1.0 / pdfPoint);

            // We also convert the pdf to solid angles.
            // In the estimator, we use the area measure pdf,
            // because we are in the area formulation estimator, but
            // for MIS, we need to use a common measure, so we will
            // convert to solid angles just for that.
            real pdfSolidAngle = pdfPoint * (dist * dist) / cosineY;

            // Notice that if we wanted to use the solid angle
            // formulation instead, we would have used the solid
            // angle pdf, and by dividing by it, we would multiply
            // the integrand by cosY / dist^2, which along with
            // cosX gives us the geometry term. So the two
            // are equivalent. 

            return AreaEstimator(radiance, weight, sample, pdfSolidAngle, 
                visibility);
        }

    
    public:

        // Calculates the estimator for a path tracer using solid angles.
        // Here wo is the direction we came from (light exits
        // there), and our goal is to calculate the emission at the
        // point of contact it, and a new sampled direction wi
        // from whence the light comes. 
        static SolidAngleEstimator sampleSolidAngle(const Vector3& wo, 
            const SurfacePoint& it){

            // We will use the BSDF at the point of contact x
            // to get the direction wi, from where the light comes,
            // as the BSDF is the only object that knows what directions
            // are valid based on the material type.
            BsdfSample sample = it.sampleBsdf(wo);
            SolidAngleEstimator estimate;

            // Otherwise, we build an estimator use the sample
            Vector3 radiance = it.evaluateEmission(wo);
            // The cosine doesn't need transformation since the shading
            // frame is orthonormal.
            Vector3 weight = sample.bsdf() * sample.cosine() 
                * (1.0 / sample.pdf());

            return SolidAngleEstimator(radiance, weight, sample.wi(),
                sample.pdf());
        }


        // Samples a point using the area formulation for a regular
        // path tracer (not NEE).
        static AreaEstimator sampleArea(const Vector3& wo, 
            const SurfacePoint& it, const Scene& scene){


        }

        
        // Samples a point using the area formulation for NEE.
        // Only returns amount of light that arrives
        static Vector3 sampleLight(const Vector3& wo, 
            const SurfacePoint& it, const Scene& scene){


        }

    };

}

#endif