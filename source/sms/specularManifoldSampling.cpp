
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
#include "texture/leanMap.hpp"
#include "core/random.hpp"
#include "logger.hpp"
#include "math/mathUtil.hpp"


namespace pathtracer{


    SpecularManifoldSampling::SpecularManifoldSampling(const Scene* scene, 
        bool useHalfVector, bool biased, bool twoStage, int maxIterations, 
        int maxTrials, int maxBiasedTrials, real epsilon, real threshold) :
        m_epsilon{epsilon},
        m_maxIterations{maxIterations},
        m_maxTrials{maxTrials},
        m_maxBiasedTrials{maxBiasedTrials},
        m_threshold{threshold},
        m_useHalfVector{useHalfVector},
        m_biased{biased},
        m_twoStage{twoStage} {

        for(int i = 0; i < scene->instanceCount(); i++){

            const SpecularManifoldBsdf* bsdf =
                dynamic_cast<const SpecularManifoldBsdf*>(scene->instance(i)->bsdf());

            // Saves only surfaces that inherit specular manifold class
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

            // We can sample a normal if it rough, else we use the
            // default (0, 0, 1) offset, which is no offset at all.
            const SpecularManifoldBsdf* bsdf = m_bsdfs[i];

            Vector3 localOffsetNormal(0, 0, 1);
            real offsetPdf = 1.0;

            if(bsdf->isRough()){
                // We assume the surface has constant roughness,
                // so we pass the center uv. It's an assumption the paper
                // makes as well.
                ManifoldNormal m = bsdf->sampleManifoldNormal(Vector2(0.5, 0.5));
                localOffsetNormal = m.normal;
                offsetPdf = m.pdf;
            }
            
            if(m_biased){
                weight = biasedSample(causticPoint, m_specularInstances[i], 
                    m_bsdfs[i], lightPoint, wo, localOffsetNormal, scene);
            }
            else{
                weight = unbiasedSample(causticPoint, m_specularInstances[i], 
                    m_bsdfs[i], lightPoint, wo, localOffsetNormal, scene);
            }

            // We divide weight by light selection probability, and the
            // offset normal probability.
            weight = weight / (lightSelectionPdf * offsetPdf);

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
        const Vector3& localOffsetNormal, const Scene* scene){

        // We sample one path at first
        SmsSample sample = samplePath(causticPoint, specular, bsdf, 
            lightPoint, localOffsetNormal, scene);

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
                specular, bsdf, lightPoint, localOffsetNormal, scene);

            Vector3 delta = newSample.finalPoint().position()
                - sample.finalPoint().position();

            if (newSample.isConverged() &&
                delta.lengthSquared() < m_threshold * m_threshold) {

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
        const Vector3& localOffsetNormal, const Scene* scene){

        Vector3 value(0.0);

        // For biased SMS, we will run it M times, and then cluster
        // the same solutions together, weight them by appropriate
        // pdfs, and use them all to get the contribution, which is biased
        // (since we use a max M), but does not waste samples and reuses
        // them for less variance.
        std::vector<Vector3> solutions;
        solutions.reserve(m_maxBiasedTrials);

        real threshold = m_threshold * m_threshold;

        for(int i = 0; i < m_maxBiasedTrials; i++){

            SmsSample sample = samplePath(causticPoint, specular, bsdf,
                lightPoint, localOffsetNormal, scene);

            if(!sample.isConverged()){ 
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

            if(!visible){ 
                continue;
            }

            // We check if this has already been discovered before
            bool duplicate = false;

            for(const Vector3& solution : solutions){
                real delta = (sample.finalPoint().position() - solution).lengthSquared();

                if(delta < threshold){
                    duplicate = true;
                    break;
                }
            }

            // Duplicates are not needed
            if(duplicate){
                continue;
            }

            // New unique solution is added otherwise
            solutions.push_back(sample.finalPoint().position());

            Vector3 weight = evaluatePathContribution(causticPoint,
                lightPoint, sample, bsdf, wo, scene);

            // Notice that we don't include the pdf of sampling this
            // point on the specular (unlike unbiased where we use a counter
            // to estimate it). This is because they cancel out (we divide
            // and multiply by number of repetitions for same converged
            // sample point).
            value = value + weight;
        }

        return value;

    }


    SmsSample SpecularManifoldSampling::samplePath(
        const SurfacePoint& causticPoint, const Instance* specular, 
        const SpecularManifoldBsdf* bsdf, const LightSample& lightPoint, 
        const Vector3& localOffsetNormal, const Scene* scene) {

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

        // Here we just test visibility
        Ray ray(causticPoint.position() + dir * SHADOW_EPSILON, dir);
        Intersection it = scene->intersect(ray);

        if(!it || it.t() < distance - 2 * SHADOW_EPSILON || it.instance() != specular){
            return SmsSample(sample, Vector3(0.0), 1.0, true, false);
        }

        SurfacePoint seedIt = it;
        
        // Need to make sure there is a normal texture
        if(m_twoStage && it.instance()->normalTexture()){
            // First we create a new instance with smooth normal map
            Intersection newIt = it;
            newIt.instance()->normalMap(newIt, true);

            // We also sample a random local offset from LEAN map
            int maxL = it.instance()->leanMap()->maxLevel(it.instance()->normalTexture());
            Vector2 mu = it.instance()->leanMap()->mean(it.uv(), maxL);
            Matrix2 sigma = it.instance()->leanMap()->covariance(it.uv(), maxL);

            Vector2 muOffset(-localOffsetNormal.x() / localOffsetNormal.z(),
                -localOffsetNormal.y() / localOffsetNormal.z());
            mu = (mu + muOffset) * 0.5;
            Vector2 slope = Gaussian::transform(Random::next2D(), mu, sigma);

            // Lean normal lives in the base frame, and we use it as our
            // offset normal.
            Vector3 leanNormal = Vector3(-slope[0], -slope[1], 1.0).normalized();

            SmsSample result = newtonSolver(causticPoint, newIt, lightPoint, bsdf,
                leanNormal, scene, true);

            // If not converged, we just return a false sample
            if(!result.isConverged()){
                m_not++;
                return result;
            }
            m_con++;

            // And we use a new seed
            seedIt = result.finalPoint();
            // Which we also normal map again to get back the original map
            it.instance()->normalMap(seedIt, false); 
        }

        // We then do a newton solver, either with initial seed or new seed
        // in two stage manifold walk.
        SmsSample result = newtonSolver(causticPoint, seedIt, lightPoint, bsdf,
            localOffsetNormal, scene, false);

        return result;
    }


    SmsSample SpecularManifoldSampling::newtonSolver(const SurfacePoint& x0, 
        const SurfacePoint& seedIt, const LightSample& x2, 
        const SpecularManifoldBsdf* bsdf, const Vector3& localOffsetNormal,
        const Scene* scene, bool smooth) {

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
            SurfaceDifferentials d = finalIt.instance()->computeDifferentials(finalIt, smooth);

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
                    halfVector, d, reflection, eta, localOffsetNormal);
            }
            else{
                step = AngleDifferenceConstraint::computeNewtonStep(x0, x2, 
                    finalIt, d, reflection, eta, localOffsetNormal);
            }

            // If newton fails we can stop
            if(!step.success){
                break;
            }

            // If the constraint is 0.0, we return a success.
            // We reuse the newton constraint instead of recomputing it.
            if(step.C.lengthSquared() < threshold){

                // We need to check that the reflection/refraction is supported
                // This is better and less noisy than letting it zero out
                // at the evaluation as it can contribute to pdf.
                if((reflection && bsdf->supportsReflection(finalIt.uv()))
                    || (!reflection && bsdf->supportsRefraction(finalIt.uv()))){

                    ++m_converged;
                    return SmsSample(finalIt, halfVector, eta, reflection, true);
                }

                // Otherwise we return false
                return SmsSample(finalIt, halfVector, eta, reflection, false);
            }

            Intersection nextIt;
            bool valid = false;

            while(!valid && k < m_maxIterations){
                Vector3 proposedPosition = finalIt.position()
                    - (d.dpdu() * step.dx.x() + d.dpdv() * step.dx.y()) * beta;

                Vector3 direction = proposedPosition - x0.position();
                direction = direction / direction.length();

                Ray ray(x0.position() + direction * SHADOW_EPSILON, direction);
                nextIt = scene->intersect(ray);

                valid = nextIt && nextIt.instance() == seedIt.instance();

                if(!valid){
                    beta *= 0.5f;
                }
                else{
                    beta = std::min(1.0, 2.0 * beta);
                }
                k++;
            }

            if(!valid){
            // We ran out of iterations, so we just leave the outer loop
                break;
            }

            // If this is stage 1 manifold walk, we can smooth the
            // intersection we get.
            if(smooth && nextIt.instance()->normalTexture()){
                nextIt.instance()->normalMap(nextIt, true);
            }

            finalIt = nextIt;
        }

        // Failure
        ++m_maxIt;
        return SmsSample(finalIt, halfVector, eta, reflection, false);
    }


    Vector3 SpecularManifoldSampling::evaluatePathContribution(
        const SurfacePoint& causticPoint, const LightSample& lightPoint, 
        const SmsSample& sample, const SpecularManifoldBsdf* bsdf, 
        const Vector3& wo, const Scene* scene) {

        Vector3 wi = (sample.finalPoint().position() 
            - causticPoint.position()).normalized();

        // First we compute bsdf at x0.
        BsdfSample s0 = causticPoint.evaluateBsdf(wo, wi);
        // No need to include the pdf since accounted for.
        Vector3 s0Weight = s0.bsdf() * s0.cosine();

        // Same for the bsdf at the specular point. However, note that
        // the bsdf at the specular can't be evaluated since it is
        // a delta distribution, so we instead use specular specific 
        // functions.

        Vector3 secondWo = -wi;
        secondWo = sample.finalPoint().shadingFrame()
            .inverseTransformDirection(secondWo);

        Vector3 secondWi = (lightPoint.position() 
            - sample.finalPoint().position()).normalized();
        secondWi = sample.finalPoint().shadingFrame()
            .inverseTransformDirection(secondWi);

        Vector3 s1Weight;
        if(sample.isReflection()){
            // This is wo from x1 to x0
            // This expects wi in local coordinates.
            // It returns the bsdf result (no cosine or pdf)
            s1Weight = bsdf->evaluateReflection(secondWo, secondWi, 
                sample.finalPoint().uv());
        }
        else{
            s1Weight = bsdf->evaluateRefraction(secondWo, secondWi, 
                sample.finalPoint().uv());
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