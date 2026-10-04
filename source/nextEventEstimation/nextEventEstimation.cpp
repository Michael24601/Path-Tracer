
#include "core/scene.hpp"
#include "nextEventEstimation/nextEventEstimation.hpp"
#include "intersection/surfacePoint.hpp"
#include "core/sceneUtil.hpp"
#include "light/light.hpp"

namespace pathtracer{

    namespace NextEventEstimation{

        // Samples a light, and returns the light sample, the contribution
        // (weighted by the solid angle pdf), and the visibility.
        // The surface point is the origin point sampling the light.
        NextEventSample sample(const Scene& scene, const SurfacePoint& it){

            // Note that NEE can't sample from emissive surfaces,
            // but specifically from lights (emissive surfaces
            // have to be considered an area light explicitly)
            if(scene.lightCount() <= 0){
                return NextEventSample{LightSample::INVALID, false, Vector3(0.0), 1.0};
            }

            // First we pick a random light with uniform probability.
            const Light* light = UniformLight::sample(scene);
            // The pdf of choosing this instance
            real pdfInstance = UniformLight::pdf(scene, light);

            // Then we sample the light for a point and radiance,
            // which returns a light sample in world coordinates
            // in solid angle measure.
            LightSample s = light->sample(it.position());

            // The visibility term (note that we don't use the light.point
            // since that may not be valid for certain lights like directional).
            bool visibility = scene.visibility(it.position(), s.wi(), s.distance());

            if(!s.isValid()){
                return NextEventSample{LightSample::INVALID, visibility, Vector3(0.0), 1.0};
            }

            // The pdf of choosing this point is the pdf of
            // choosing the light times the pdf of choosing the point
            // on the light.
            real pdfPoint = pdfInstance * s.pdf();

            Vector3 contribution = s.radiance() / pdfPoint;

            return NextEventSample{s, visibility, contribution, pdfPoint};

        }

    }

}