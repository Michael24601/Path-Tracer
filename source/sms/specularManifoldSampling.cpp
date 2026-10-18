
#include "sms/specularManifoldSampling.hpp"
#include <algorithm>
#include <cmath>
#include "core/ray.hpp"
#include "intersection/intersection.hpp"
#include "shapes/shape.hpp"
#include "math/constants.hpp"
#include "bsdf/specularManifoldBsdf.hpp"
#include "core/scene.hpp"
#include "sms/smsSample.hpp"
#include "light/lightSample.hpp"
#include "intersection/areaSample.hpp"
#include "sms/smsUtil.hpp"
#include "intersection/surfaceDifferentials.hpp"
#include "core/instance.hpp"
#include "light/light.hpp"
#include "bsdf/bsdfSample.hpp"
#include "bsdf/bsdf.hpp"
#include "core/sceneUtil.hpp"
#include "sms/halfVectorConstraint.hpp"
#include "sms/angleDifferenceConstraint.hpp"
#include "logger.hpp"
#include <iostream>

namespace pathtracer{


    SpecularManifoldSampling::SpecularManifoldSampling(const Scene* scene, 
        bool useHalfVector, bool biased, int maxIterations, int maxTrials, 
        int maxBiasedTrials, real epsilon, real threshold) :
        m_epsilon{epsilon},
        m_maxIterations{maxIterations},
        m_maxTrials{maxTrials},
        m_maxBiasedTrials{maxBiasedTrials},
        m_threshold{threshold},
        m_useHalfVector{useHalfVector},
        m_biased{biased} {

        for(int i = 0; i < scene->instanceCount(); i++){

            const SpecularManifoldBsdf* bsdf =
                dynamic_cast<const SpecularManifoldBsdf*>(scene->instance(i)->bsdf());

            if(bsdf){
                m_specularInstances.push_back(scene->instance(i));
                m_bsdfs.push_back(bsdf);
            }
        }
    }


    Vector3 SpecularManifoldSampling::sample(const SurfacePoint& causticPoint, 
        const Vector3& wo, const Scene* scene) {

        // First the light is sampled.
        const Light* light = UniformLight::sample(*scene);
        LightSample lightPoint = light->sample(causticPoint.position());
        real lightSelectionPdf = UniformLight::pdf(*scene, light);

        if(!lightPoint.isValid()){
            return Vector3(0.0);
        }

        Vector3 totalColor(0.0);

        // First we loop over all shapes and search until we find
        // a shape with a valid SMS sample.
        for(int i = 0; i < m_specularInstances.size(); i++){

            Vector3 weight; 
            
            if(m_biased){
                weight = biasedSample(causticPoint, m_specularInstances[i], 
                    m_bsdfs[i], lightPoint, wo, scene);
            }
            else{
                weight = unbiasedSample(causticPoint, m_specularInstances[i], 
                    m_bsdfs[i], lightPoint, wo, scene);
            }

            // We divide weight by light selection probability
            weight = weight / lightSelectionPdf;

            // We add up the weight from each specular
            totalColor = totalColor + weight;
        }

        // If we get here, it is a failure, and we return 0.0
        return totalColor;
    }



    Vector3 SpecularManifoldSampling::unbiasedSample(
        const SurfacePoint& causticPoint,
        const Instance* specular, const SpecularManifoldBsdf* bsdf,
        const LightSample& lightPoint, const Vector3& wo, 
        const Scene* scene){

        // We sample one path at first
        SmsSample sample = samplePath(causticPoint, specular, bsdf, 
            lightPoint, scene);

        // If not valid, we discard this path
        if(!sample.isConverged()){
            return Vector3(0.0);
        }

        // We know x0 to x1 is visible (samplePath checks),
        // but now we check x1 to x2.
        // It depends on if it is directional or not.
        bool visible;
        if(lightPoint.caster()->isDirectional()){
            visible = scene->visibility(sample.finalPoint().position(),
                lightPoint.wi(), lightPoint.distance());
        }
        else{
            visible = scene->visibility(sample.finalPoint().position(),
                lightPoint.position());
        }

        if(!visible){
            m_notVis++;
            return Vector3(0.0);
        }

        // If all is valid, we can calculate the path contribution.
        int counter = 0;
        bool found = false;

        // Direction from x1 to x0
        Vector3 dir = (sample.finalPoint().position()
            - causticPoint.position()).normalized();

        while(counter < m_maxTrials){

            counter++;

            SmsSample newSample = samplePath(causticPoint,
                specular, bsdf, lightPoint, scene);

            Vector3 newDir = (newSample.finalPoint().position()
                - causticPoint.position()).normalized();
            real angularDist = newDir.dot(dir);

            if(newSample.isConverged() && std::abs(angularDist - 1) < m_threshold){
                found = true;
                m_sameFound++;
                break;
            }
        }

        // Failure (if none are found)
        if(!found){
            m_sameNotFound++;
            return Vector3(0.0);
        }

        // Otherwise we can compute the probability of sampling x1.
        // Based on the fact that this a geometric distribution,
        // the estimate for p is 1 / number of trials until
        // we get the same converged vertex, which means 1/p
        // is estimated by the counter.
        real invProbability = static_cast<real>(counter);

        // Then we can evaluate the path contribution
        // which includes the bsdf at x0 and x1, the cosine terms,
        // the pdf for sampling x2... all except the pdf of x1.
        Vector3 weight = evaluatePathContribution(causticPoint,
            lightPoint, sample, bsdf, wo, scene);

        // We add both the invProbability for x1.
        weight = weight * invProbability;

        return weight;
    }



    Vector3 SpecularManifoldSampling::biasedSample(
        const SurfacePoint& causticPoint,
        const Instance* specular, const SpecularManifoldBsdf* bsdf,
        const LightSample& lightPoint, const Vector3& wo, 
        const Scene* scene){

        Vector3 value(0.0);

        // For biased SMS, we will run it M times, and then cluster
        // the same solutions together, weight them by appropriate
        // pdfs, and use them all to get the contribution, which is biased
        // (since we use a max M), but does not waste samples and reuses
        // them for less variance.
        std::vector<Vector3> solutions;
        solutions.reserve(m_maxBiasedTrials);

        int nc = 0;
        int nv = 0;
        int s = 0;

        for(int i = 0; i < m_maxBiasedTrials; i++){

            SmsSample sample = samplePath(causticPoint, specular, bsdf,
                lightPoint, scene);

            if(!sample.isConverged()){ nc++;
                continue;
            }

            // We know x0 to x1 is visible (samplePath checks),
            // but now we check x1 to x2.
            bool visible;
            if(lightPoint.caster()->isDirectional()){
                visible = scene->visibility(sample.finalPoint().position(),
                    lightPoint.wi(), lightPoint.distance());
            }
            else{
                visible = scene->visibility(sample.finalPoint().position(),
                    lightPoint.position());
            }

            if(!visible){ nv++;
                m_notVis++;
                continue;
            }

            // Direction from x0 to x1.
            Vector3 dir = (sample.finalPoint().position()
                - causticPoint.position()).normalized();

            // We check if this has already been discovered before
            bool duplicate = false;

            for(const Vector3& solution : solutions){
                real angularDist = dir.dot(solution);

                if(std::abs(angularDist - 1.0) < m_threshold){
                    duplicate = true;
                    break;
                }
            }

            // Duplicates are not needed
            if(duplicate){ s++;
                continue;
            }

            // New unique solution is added otherwise
            solutions.push_back(dir);

            Vector3 weight = evaluatePathContribution(causticPoint,
                lightPoint, sample, bsdf, wo, scene);

            // Notice that we don't include the pdf of sampling this
            // point on the specular (unlike unbiased where we use a counter
            // to estimate it). This is because they cancel out (we divide
            // and multiply by number of repetitions for same converged
            // sample point).
            value = value + weight;
        }

        //LOG_INFO(std::to_string(nc) + " " + std::to_string(nv) + " " + std::to_string(s))
        //LOG_INFO(std::to_string(solutions.size()));

        return value;

    }



    SmsSample SpecularManifoldSampling::samplePath(
        const SurfacePoint& causticPoint, const Instance* specular, 
        const SpecularManifoldBsdf* bsdf, const LightSample& lightPoint, 
        const Scene* scene) {

        // We will have to first sample a point on the specular surface
        // which we can do using areaSampling.
        // The position that was sampled is the seed.
        // This sample is itself a surfacePoint so no need to ray trace.
        AreaSample sample = specular->sampleArea();

        // Next up, we need to perform newton-iterations to find
        // a valid point. Note that the newton solver will use the
        // angles with the seed point to decide if it is a refraction
        // or a reflection.

        // Here could be a good place to possibly sample another seed
        // point in case the sample point causes a refraction on
        // a reflection only bsdf for example.
        // Regardless, if it fails, the evaluation of the bsdf
        // later in the code will zero out the contribution,
        // so this is not solving a bug but instead avoid wasting
        // a sample.

        // We will first check the visibility of the sampled point
        // from x0. If not visible, we can skip newton entirely
        // and return false.
        Vector3 dir = (sample.position() - causticPoint.position());

        real distance = dir.length();
        dir = dir / distance;

        Ray ray(causticPoint.position() + dir * SHADOW_EPSILON, dir);
        Intersection it = scene->intersect(ray);

        if(!it || it.t() < distance - 2 * SHADOW_EPSILON || it.instance() != specular){
            return SmsSample(sample, sample, Vector3(0.0), 1.0, true, false);
        }

        // Otherwise we use the newton solver
        SmsSample result = newtonSolver(causticPoint, it, lightPoint, bsdf, scene);
        return result;
    }


    SmsSample SpecularManifoldSampling::newtonSolver(const SurfacePoint& x0, 
        const SurfacePoint& seedIt, const LightSample& x2, 
        const SpecularManifoldBsdf* bsdf, const Scene* scene) {

        // We use the newton solver to converge to a point on the
        // instance that satisfies the chosen specular constraint
        // (first or second).

        // We square it so we don't need to use square root later
        real threshold = m_epsilon * m_epsilon;

        // Initially the sample is just the seed.
        SurfacePoint finalIt = seedIt;

        // Values we need after loop
        Vector3 halfVector;
        real eta;
        bool reflection;

        // Adaptive step size
        real beta = 1.0;

        // We then run newton solver
        int k{0};
        while(k < m_maxIterations){

            // First we compute intermediate values

            Vector3 wo = (x0.position() - finalIt.position()).normalized();

            Vector3 wi;
            if(x2.caster()->isDirectional()){
                wi = x2.wi();
            }
            else{
                wi = (x2.position() - finalIt.position()).normalized();
            }

            reflection = SmsUtil::isReflection(wo, finalIt, wi);
            Vector3 localWo = finalIt.shadingFrame().inverseTransformDirection(wo);
            eta = bsdf->eta(localWo, finalIt.uv());
            SurfaceDifferentials d = finalIt.instance()->computeDifferentials(finalIt);

            // Notice that half vector is updated regardless of which
            // constraint we use as we need it for the geometric term anyway.
            halfVector = SmsUtil::halfVector(wo, finalIt, wi, eta, reflection);

            // We do newton step and reproject. The new position
            // is on the tangent space of the specular surface at the
            // previous proposition.

            // The offset from newton
            NewtonOutput step; 
            if(m_useHalfVector){
                step = HalfVectorConstraint::computeNewtonStep(x0, x2, finalIt, 
                    halfVector, d, reflection, eta);
            }
            else{
                step = AngleDifferenceConstraint::computeNewtonStep(x0, x2, 
                    finalIt, d, reflection, eta);
            }

            // If newton fails we can stop
            if(!step.success){
                break;
            }

            // If the constraint is 0.0, we return a success.
            // We reuse the newton constraint instead of recomputing it.
            if(step.C.lengthSquared() < threshold){
                ++m_converged;
                return SmsSample(seedIt, finalIt, halfVector, eta, reflection, true);
            }

            // Otherwise, we need to update the position.
            Vector3 proposedPosition = finalIt.position()
                -(d.dpdu() * step.dx.x() + d.dpdv() * step.dx.y()) * beta;

            Vector3 direction = (proposedPosition - x0.position());
            real distance = direction.length();
            direction = direction/ distance;

            // We then reproject
            Ray ray(x0.position() + direction * SHADOW_EPSILON, direction);
            Intersection nextIt = scene->intersect(ray);

            // Note: we don't check visibility, since the point
            // is not projected yet (distance mismatch). It is enough to 
            // check if it is the same instance.
            if(!nextIt || nextIt.instance() != seedIt.instance()){

                // The step size was likely too large, so we make
                // the step size smaller (adaptive), and try again.
                beta *= 0.5f;
                k++;
                continue;
            }

            // If successful, beta is reset
            beta = std::min(1.0, 2.0 * beta);
           
            // And then we repeat
            finalIt = nextIt;
            k++;
        }

        // Failure
        ++m_maxIt;
        return SmsSample(seedIt, finalIt, halfVector, eta, reflection, false);
    }


    Vector3 SpecularManifoldSampling::evaluatePathContribution(
        const SurfacePoint& causticPoint, const LightSample& lightPoint, 
        const SmsSample& sample, const SpecularManifoldBsdf* bsdf, 
        const Vector3& wo, const Scene* scene) {

        Vector3 wi = (sample.finalPoint().position() - causticPoint.position()).normalized();

        // First we compute bsdf at x0.
        BsdfSample s0 = causticPoint.evaluateBsdf(wo, wi);
        // No need to include the pdf since accounted for.
        Vector3 s0Weight = s0.bsdf() * s0.cosine();

        // Same for the bsdf at the specular point. However, note that
        // the bsdf at the specular can't be evaluated since it is
        // a delta distribution, so we instead use specular specific 
        // functions.
        Vector3 s1Weight;
        if(sample.isReflection()){
            // This is wo from x1 to x0
            Vector3 secondWo = -wi;
            secondWo = sample.finalPoint().shadingFrame().inverseTransformDirection(secondWo);
            // This expects wi in local coordinates.
            // It returns the bsdf result (no cosine or pdf)
            s1Weight = bsdf->evaluateReflection(secondWo, sample.finalPoint().uv());
        }
        else{
            Vector3 secondWo = -wi;
            secondWo = sample.finalPoint().shadingFrame().inverseTransformDirection(secondWo);
            s1Weight = bsdf->evaluateRefraction(secondWo, sample.finalPoint().uv());
        }

        // This already includes the light pdf (in case of area light).
        SmsLightSample lightSample = SmsUtil::lightContribution(lightPoint, 
            sample.finalPoint().position());

        // Next up, we need the jacobian (the differential frame must be made
        // orthonormal).

        SurfaceDifferentials d0 = causticPoint.instance()
            ->computeDifferentials(causticPoint);
        d0.makeOrthonormal();

        SurfaceDifferentials d1 = sample.finalPoint().instance()
            ->computeDifferentials(sample.finalPoint());
        d1.makeOrthonormal();

        // The other one, at a light
        SurfaceDifferentials d2 = lightSample.sample.caster()
            ->computeDifferentials(lightSample.sample);

        // We only make it orthonormal for area light
        if(lightSample.sample.caster()->hasArea()){
            d2.makeOrthonormal();
        }

        real g = SmsUtil::geometricTerm(causticPoint, sample, 
            lightSample.sample, d0, d1, d2);

        return lightSample.contribution * s1Weight * s0Weight * g;
    }

}