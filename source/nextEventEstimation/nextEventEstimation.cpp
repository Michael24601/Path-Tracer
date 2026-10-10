
#include "core/scene.hpp"
#include "nextEventEstimation/nextEventEstimation.hpp"
#include "intersection/surfacePoint.hpp"
#include "core/sceneUtil.hpp"
#include "core/instance.hpp"
#include "light/light.hpp"
#include "shapes/shape.hpp"
#include "intersection/areaSample.hpp"
#include "logger.hpp"

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


        NextEventSample evaluate(const Scene& scene, const Vector3& origin,
            const SurfacePoint& emissive){

            // The instance has to be emissive, a light (sampleable by NEE),
            // and an area light that a bsdf can intersect.
            if(!emissive.instance()->emission() || !emissive.instance()->light()
                || !emissive.instance()->light()->isIntersectable()){

                return NextEventSample{LightSample::INVALID, false, Vector3(0.0), 0.0};
            }

            // We can call the evaluate of the light (only works if light
            // is intersectable).
            SurfaceSample s{emissive.position(), emissive.triangleIndex()};
            LightSample sample = emissive.instance()->light()
                ->evaluateLightSample(origin, s);

            // We also multiply p_nee by probability of choosing
            // said light.
            real pdf = sample.pdf() * UniformLight::pdf(scene, 
                emissive.instance()->light());
            
            bool visibility = scene.visibility(origin, sample.wi(), sample.distance());

            Vector3 contribution = sample.radiance() / pdf;
            
            return NextEventSample{sample, visibility, contribution, pdf};
        }

    }

}