
#ifndef PATH_TRACER_SPECULAR_MANIFOLD_SAMPLING_HPP
#define PATH_TRACER_SPECULAR_MANIFOLD_SAMPLING_HPP

#include "../core/instance.hpp"
#include "../core/scene.hpp"
#include "../bsdf/specularBsdf.hpp"
#include "smsSample.hpp"
#include "smsUtil.hpp"
#include "../intersection/areaSample.hpp"
#include "../light/lightSample.hpp"

namespace pathtracer{

    // Single scatter event SMS
    class SpecularManifoldSampling{

    private:

        // Keeps a pointer to specular instances
        std::vector<const Instance*> m_specularInstances;

        // In order to avoid repeated downcasting calls,
        // the specular bsdfs are cached here (instance holds pointer
        // to base class).
        // The downcast is necessary for specular specific functions.
        std::vector<const SpecularBsdf*> m_bsdfs;


        // Maximum numbers of allowed iterations for the newton solver
        int m_maxIterations;

        // Maximum trials allowed to compute inverse probability
        int m_maxTrials;

        // The threshold for two samples being the same
        real m_threshold;

        // The threshold for checking if the specular constraint is met.
        real m_epsilon;

    public:


        std::atomic<int> m_converged{0};
        std::atomic<int> m_maxIt{0};

        std::atomic<int> m_sameFound{0};
        std::atomic<int> m_sameNotFound{0};

        std::atomic<int> m_notVis{0};


        SpecularManifoldSampling(const Scene* scene, int maxIterations = 100,
            int maxTrials = 1000, real epsilon = 1e-5, real threshold = 1e-4) : 
            m_epsilon{epsilon}, m_maxIterations{maxIterations}, 
            m_maxTrials{maxTrials}, m_threshold{threshold}{

            for(int i = 0; i < scene->instanceCount(); i++){

                const SpecularBsdf* bsdf = 
                    dynamic_cast<const SpecularBsdf*>(scene->instance(i)->bsdf());

                if(bsdf){
                    m_specularInstances.push_back(scene->instance(i));
                    m_bsdfs.push_back(bsdf);
                }
            }
        }


        // Returns the outgoing radiance from x0, after connecting
        // x0 to a specular point x1, connected to x2 on the light, 
        // satisfying the specular constraint.
        // The wo pointing away from x0 to the previous point is also sent.
        Vector3 sample(const SurfacePoint& causticPoint, 
            const LightSample& lightPoint, const Vector3& wo,
            real lightSelectionPdf, const Scene* scene, bool& success){

            success = false;

            // First we loop over all shapes and search until we find
            // a shape with a valid SMS sample.
            for(int i = 0; i < m_specularInstances.size(); i++){

                SmsSample sample = samplePath(causticPoint.position(), 
                    m_specularInstances[i], m_bsdfs[i], 
                    lightPoint.position(), scene);

                // If not valid, we continue to next shape
                if(!sample.isConverged()){
                    continue;
                }

                // We know x0 to x1 is visible (samplePath checks),
                // but now we check x1 to x2.
                bool visible = scene->visibility(sample.finalPoint().position(), 
                    lightPoint.position());

                if(!visible){
                    m_notVis++;
                    continue;
                }


                // If all is valid, we can calculate the path contribution.
                int counter = 0;
                bool found = false;

                // Direction from x1 to x0
                Vector3 dir = (sample.finalPoint().position() 
                    - causticPoint.position()).normalized();

                while(counter < m_maxTrials){

                    counter++;
                    
                    SmsSample newSample = samplePath(causticPoint.position(), 
                    m_specularInstances[i], m_bsdfs[i], 
                    lightPoint.position(), scene);

                    Vector3 newDir = (newSample.finalPoint().position() 
                        - causticPoint.position()).normalized();
                    real angularDist = newDir.dot(dir);

                    if(newSample.isConverged() && std::abs(angularDist - 1) < m_threshold){
                        found = true;
                        m_sameFound++;
                        break;
                    }
                }

                // Failure
                if(!found){
                    m_sameNotFound++;
                    continue;
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
                    lightPoint, sample, m_bsdfs[i], wo, lightSelectionPdf, scene);
                
                weight = weight * invProbability;

                success = true;
                return weight;
            }

            success = false;
            // If we get here, it is a failure, and we return 0.0
            return Vector3(0.0);

        }


        // Given a point on a light x2 (that we already 
        // supposedly sampled outside), a point on a surface received a 
        // caustic x0, and a specular instance in between, this function
        // constructs a path through the specular surface by finding a
        // point x1 on it that satisfies the specular constraint.
        // It returns the seed point, and the converged point (after newton's
        // method is used to satisfy the constraints). May return
        // a non converged result.
        SmsSample samplePath(const Vector3& causticPoint, 
            const Instance* specular, const SpecularBsdf* bsdf, 
            const Vector3& lightPoint, const Scene* scene){

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
            Vector3 dir = (sample.position() - causticPoint);

            real distance = dir.length();
            dir = dir / distance;

            Ray ray(causticPoint + dir * SHADOW_EPSILON, dir);
            Intersection it = scene->intersect(ray);

            if(!it || it.t() < distance - 2 * SHADOW_EPSILON || it.instance() != specular){
                return SmsSample(sample, sample, Vector3(0.0), 1.0, true, false);
            }

            // Otherwise we use the newton solver
            SmsSample result = newtonSolver(causticPoint, it, lightPoint, bsdf, scene);
            return result;
        }


        SmsSample newtonSolver(const Vector3& x0, 
            const SurfacePoint& seedIt, const Vector3& x2, 
            const SpecularBsdf* bsdf, const Scene* scene){

            // We use the newton solver to converge to a point on the
            // instance that satisfies the chosen specular constraint
            // (first or second).

            // We square it so we don't need to use square root later
            real threshold = m_epsilon * m_epsilon;

            Vector3 wo = (x0 - seedIt.position()).normalized();
            Vector3 wi = (x2 - seedIt.position()).normalized();
            
            bool reflection = SmsUtil::isReflection(wo, seedIt, wi);

            // We need local wo
            Vector3 localWo = seedIt.shadingFrame().inverseTransformDirection(wo);
            real eta = bsdf->eta(localWo, seedIt.uv());
            SurfaceDifferentials d = seedIt.instance()->computeDifferentials(seedIt);

            // Note, not normalized
            Vector3 halfVector = SmsUtil::halfVector(wo, seedIt, wi, eta, reflection);
            Vector2 c = SmsUtil::specularConstraint(seedIt, halfVector, d);

            // First we check if the point satisfies the constraints.
            // If so no need to run the solver.
            if(c.lengthSquared() < threshold){
                ++m_converged;
                return SmsSample(seedIt, seedIt, halfVector, eta, reflection, true);
            }

            // Initially the sample is just the seed.
            SurfacePoint finalIt = seedIt;

            // Adaptive step size
            real beta = 1.0;

            // We then run newton solver
            int k{0};
            while(k < m_maxIterations){

                // First we propose a new position. This position
                // is on the tangent space of the specular surface at the
                // previous proposition.

                Vector2 dx = SmsUtil::computeNewtonStep(x0, x2, finalIt,
                    wo, wi, halfVector, d, reflection, eta);

                // We then offset and ray trace
                Vector3 proposedPosition = finalIt.position()
                    -(d.dpdu() * dx[0] + d.dpdv() * dx[1]) * beta;

              
                Vector3 direction = (proposedPosition - x0);
                real distance = direction.length();
                direction = direction/ distance;
                
                Ray ray(x0 + direction * SHADOW_EPSILON, direction);
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
                
                // Otherwise we check if constraints are met.
                // but first we recompute some values.

                wo = (x0 - nextIt.position()).normalized();
                wi = (x2 - nextIt.position()).normalized();
            
                reflection = SmsUtil::isReflection(wo, nextIt, wi);
                localWo = nextIt.shadingFrame().inverseTransformDirection(wo);
                eta = bsdf->eta(localWo, nextIt.uv());
                halfVector = SmsUtil::halfVector(wo, nextIt, wi, eta, reflection);

                d = nextIt.instance()->computeDifferentials(nextIt);

                c = SmsUtil::specularConstraint(nextIt, halfVector, d);
                // If they are, we return a success
                if(c.lengthSquared() < threshold){
                    ++m_converged;
                    return SmsSample(seedIt, nextIt, halfVector, eta, reflection, true);
                }

                // Otherwise we repeat
                finalIt = nextIt;

                k++;
            }

            // Failure
            ++m_maxIt;
            return SmsSample(seedIt, finalIt, halfVector, eta, reflection, false);
        }


        // Returns the ougoing contribution from x0 after it samples
        // x1 and x2 on the light. This will include all weight terms
        // except for the pdf at x1, which is computed outside
        // (since it can fail, so the main function handles it).
        static Vector3 evaluatePathContribution(
            const SurfacePoint& causticPoint, const LightSample& lightPoint, 
            const SmsSample& sample, const SpecularBsdf* bsdf, 
            const Vector3& wo, real lightSelectionPdf, const Scene* scene){

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

            // We need the light sample contribution, with respect to the new point.
            SurfaceSample s{lightPoint.position(), lightPoint.triangleIndex()};
            LightSample newSample = lightPoint.caster()->evaluateLightSample(
                sample.finalPoint().position(), s);
                
            Vector3 lightContribution = newSample.radiance();

            // For the light pdf, we can use either the one from newSample
            // or the original sample, since they are the same.
            real lightPdf = (newSample.pdf() * lightSelectionPdf);

            // Since the geometric term already handles solid angle
            // conversion, we need to undo it.
            if(lightPoint.caster()->hasArea()){
                // The conversion is done using newSample, since it depends
                // on cosine and distance which is different in the original
                // sample.
                lightPdf *= (newSample.cosine() 
                    / (newSample.distance() * newSample.distance()));
            }


             // Next up, we need the jacobian

            SurfaceDifferentials d1 = sample.finalPoint().instance()
                ->computeDifferentials(sample.finalPoint());
            d1.makeOrthonormal();

            // The other one, at a light
            SurfaceDifferentials d2 = newSample.caster()->computeDifferentials(newSample);
            d2.makeOrthonormal();

            real g = SmsUtil::geometricTerm(causticPoint, sample, newSample, d1, d2);

            return lightContribution * s1Weight * s0Weight * g / lightPdf;
            
        }



    };

}


#endif