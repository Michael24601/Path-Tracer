
#ifndef PATH_TRACER_SPECULAR_MANIFOLD_SAMPLING_HPP
#define PATH_TRACER_SPECULAR_MANIFOLD_SAMPLING_HPP

#include "../core/instance.hpp"
#include "../core/scene.hpp"
#include "../bsdf/specularBsdf.hpp"
#include "smsSample.hpp"
#include "smsUtil.hpp"
#include "../intersection/areaSample.hpp"

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

        // The threshold for checking if the specular constraint is met.
        real m_epsilon;

    public:


        SpecularManifoldSampling(const Scene* scene, int maxIterations = 20,
            real epsilon = 1e-5) : m_epsilon{epsilon},
            m_maxIterations{maxIterations}{

            for(int i = 0; i < scene->instanceCount(); i++){

                const SpecularBsdf* bsdf = 
                    dynamic_cast<const SpecularBsdf*>(scene->instance(i)->bsdf());

                if(bsdf){
                    m_specularInstances.push_back(scene->instance(i));
                    m_bsdfs.push_back(bsdf);
                }
            }
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

            LOG_INFO(dir.toString());

            real distance = dir.length();
            dir = dir / distance;

            Ray ray(causticPoint + dir * SHADOW_EPSILON, dir);
            Intersection it = scene->intersect(ray);

            if(!it || it.t() < distance - 2 * SHADOW_EPSILON || it.instance() != specular){
                return SmsSample(sample.position(), sample.position(), false);
            }

            // Otherwise we use the newton solver
            SmsSample result = newtonSolver(causticPoint, it, lightPoint, bsdf, scene);
            return result;
        }


        SmsSample newtonSolver(const Vector3& x0, 
            const Intersection& seedIt, const Vector3& x2, 
            const SpecularBsdf* bsdf, const Scene* scene){

            // We use the newton solver to converge to a point on the
            // instance that satisfies the chosen specular constraint
            // (first or second).

            // We square it so we don't need to use square root later
            real threshold = m_epsilon * m_epsilon;

            Vector3 wo = (x0 - seedIt.position()).normalized();
            Vector3 wi = (x2 - seedIt.position()).normalized();
            
            bool reflection = SmsUtil::isReflection(wo, seedIt, wi);

            real eta = bsdf->eta(wo, seedIt.uv());
            Vector3 halfVector = SmsUtil::halfVector(wo, seedIt, wi, eta, reflection);
            Vector2 c = SmsUtil::specularConstraint(seedIt, halfVector);
            
            LOG_INFO("Sample: " + seedIt.position().toString());
            LOG_INFO("C(x): " + c.toString());

            // First we check if the point satisfies the constraints.
            // If so no need to run the solver.
            if(c.lengthSquared() < threshold){
                return SmsSample(seedIt.position(), seedIt.position(), true);
            }

            // Initially the sample is just the seed.
            SurfacePoint finalIt = seedIt;

            // We then run newton solver
            int k{0};
            while(k < m_maxIterations){

                // First we propose a new position. This position
                // is on the tangent space of the specular surface at the
                // previous proposition.

                SurfaceDifferentials d = finalIt.instance()->computeDifferentials(finalIt);

                Vector2 dx = SmsUtil::computeNewtonStep(x0, x2, finalIt,
                    wo, wi, halfVector, d, reflection, eta) * 0.5;

                // We then offset and ray trace
                Vector3 proposedPosition = finalIt.position()
                    - d.dpdu() * dx[0] - d.dpdv() * dx[1];

                Vector3 direction = (proposedPosition - x0);
                real distance = direction.length();
                direction = direction/ distance;

                Ray ray(x0 + direction * SHADOW_EPSILON, direction);
                Intersection nextIt = scene->intersect(ray);

                LOG_INFO("Sample: " + nextIt.position().toString());

                // If occluded or leaves scene
                if(!nextIt || nextIt.t() < distance - SHADOW_EPSILON){
                    return SmsSample(seedIt.position(), nextIt.position(), false);
                }

                // Otherwise we check if constraints are met.
                // but first we recompute some values.

                wo = (x0 - nextIt.position()).normalized();
                wi = (x2 - nextIt.position()).normalized();
            
                reflection = SmsUtil::isReflection(wo, nextIt, wi);
                eta = bsdf->eta(wo, nextIt.uv());
                halfVector = SmsUtil::halfVector(wo, nextIt, wi, eta, reflection);

                c = SmsUtil::specularConstraint(nextIt, halfVector);

                LOG_INFO("C(x): " + c.toString());
                LOG_INFO("C(x).length^2: " + std::to_string(c.lengthSquared()));

                // If they are, we return a success
                if(c.lengthSquared() < threshold){
                    return SmsSample(seedIt.position(), nextIt.position(), true);
                }

                // Otherwise we repeat
                finalIt = nextIt;

                k++;
            }

            // Failure
            return SmsSample(seedIt.position(), finalIt.position(), false);
        }



    };

}


#endif