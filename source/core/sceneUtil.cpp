
#include "core/sceneUtil.hpp"
#include "core/instance.hpp"
#include "core/random.hpp"
#include "light/light.hpp"
#include "core/scene.hpp"


namespace pathtracer{

    // Samples an instance uniformly
    namespace UniformInstance{

        const Instance* sample(const Scene& scene) {
            int instanceCount = scene.instanceCount();
            int randomNum = static_cast<int>(
                Random::next() * instanceCount);

            const Instance* inst = scene.instance(randomNum);
            return inst;
        }


        // Pdf of having sampled this instance
        real pdf(const Scene& scene, const Instance* instance) {
            return 1.0 / scene.instanceCount();
        }

    }


    // Samples a light uniformly
    namespace UniformLight{

        const Light* sample(const Scene& scene) {
            int lightCount = scene.lightCount();
            int randomNum = static_cast<int>(
                Random::next() * lightCount);

            const Light* light = scene.light(randomNum);
            return light;
        }


        // Pdf of having sampled this light
        real pdf(const Scene& scene, const Light* light) {
            return 1.0 / scene.lightCount();
        }

    }

}