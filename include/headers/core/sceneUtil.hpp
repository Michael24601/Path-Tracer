#ifndef PATH_TRACER_SCENE_UTIL_HPP
#define PATH_TRACER_SCENE_UTIL_HPP

#include "config.hpp"

namespace pathtracer{

    // Forward declaration
    class Scene;
    class Light;
    class Instance;


    // Samples an instance uniformly
    namespace UniformInstance{

        const Instance* sample(const Scene& scene);

        // Pdf of having sampled this instance
        real pdf(const Scene& scene, const Instance* instance);

    }


    // Samples a light uniformly
    namespace UniformLight{

        const Light* sample(const Scene& scene);

        // Pdf of having sampled this light
        real pdf(const Scene& scene, const Light* light);

    }

}

#endif