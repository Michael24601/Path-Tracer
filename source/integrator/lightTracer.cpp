
#include "integrator/lightTracer.hpp"
#include "core/ray.hpp"
#include "core/scene.hpp"
#include "intersection/intersection.hpp"
#include "math/vector3.hpp"
#include "bsdf/bsdfSample.hpp"
#include "core/random.hpp"
#include "math/mathUtil.hpp"
#include "camera/camera.hpp"
#include "core/instance.hpp"
#include "math/constants.hpp"
#include "core/sceneUtil.hpp"
#include "bsdf/bsdf.hpp"
#include "light/light.hpp"
#include "logger.hpp"


namespace pathtracer{


    void LightTracer::tracePath(const Scene& scene, const Camera& camera,
        std::vector<std::vector<Vector3>>& image) {

        const Light* light = UniformLight::sample(scene);
        EmissionSample sample = light->sampleEmission(scene);

        real lightPdf = UniformLight::pdf(scene, light);

        Vector3 throughput = sample.intensity / lightPdf;

        Ray currRay(sample.point + sample.direction * SHADOW_EPSILON,
            sample.direction);

        for(int depth = 0; depth < m_maxDepth; depth++){

            if(g_stopRendering){
                break;
            }

            Vector3 wo = -currRay.direction();

            m_bounceCount++;

            Intersection it = scene.intersect(currRay);
            if(!it) break;

            if(it.instance()->emission()){
                break;
            }

            // We can't do a camera path at a specular surface as only
            // one outgoing direction is correct
            if(!it.instance()->bsdf()->isSpecular()){

                // We need to evaluate bsdf here to add the last
                // bsdf weight.

                // We need direction towards camera
                CameraConnection connection = camera.getConnection(it.position());
                
                // No need to waste time evaluating bsdf or testing
                // visibility if the point is not visible by the camera,
                // or out of bounds of the camera plane, which is what valid
                // contains.
                if(connection.valid && scene.visibility(it.position(), 
                    connection.direction, connection.distance)){

                    Vector3 wi = connection.direction;

                    // Adjoint is set to true as these are light paths
                    BsdfSample eval = it.evaluateBsdf(wo, wi, true);

                    // Instead of using the full weight, we use only
                    // the cosine and bsdf, since pdf is not needed
                    // (this part is just a projection e.g. deterministic).
                    Vector3 contribution = throughput * eval.bsdf()
                        * eval.cosine() * connection.importance;

                    // Then we add contribution to image
                    splat(contribution, connection, image);
                }
            }

            // We set adjoint to true since the path starts from the light
            BsdfSample bsdfSample = it.sampleBsdf(wo, true);
            Vector3 weight = bsdfSample.weight();

            real p = Util::russianRoulette(throughput, depth);
            if(Random::next() > p) break;

            throughput = throughput * weight * (1.0f / p);

            currRay = Ray(
                it.position() + bsdfSample.wi() * SHADOW_EPSILON,
                bsdfSample.wi()
            );
        }
    }


    void LightTracer::printStatistics(int width, int height) const {

        long long total = m_lightPaths;
        real averageBounceCount = static_cast<real>(m_bounceCount) / total;

        LOG_DEBUG("Bounce count: " + std::to_string(m_bounceCount));
        LOG_DEBUG("Average bounce count: " + std::to_string(averageBounceCount));
    }
    

}