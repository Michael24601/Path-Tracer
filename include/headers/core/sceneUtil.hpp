


#ifndef PATH_TRACER_SCENE_UTIL_HPP
#define PATH_TRACER_SCENE_UTIL_HPP

#include "scene.hpp"

namespace pathtracer{

    // Samples an instance uniformly
    class UniformInstance{

    public:

        static const Instance* sample(const Scene& scene){
            int instanceCount = scene.instanceCount();
            int randomNum = static_cast<int>(Random::next() * instanceCount);
            const Instance* inst = scene.instance(randomNum);
            return inst;
        }


        // Pdf of having sampled this instance
        static real pdf(const Scene& scene, const Instance* instance){
            return 1.0 / scene.instanceCount();
        }
    };


    // Samples a light uniformly
    class UniformLight{

    public:

        static const Light* sample(const Scene& scene){
            int lightCount = scene.lightCount();
            int randomNum = static_cast<int>(Random::next() * lightCount);
            const Light* light = scene.light(randomNum);
            return light;
        }


        // Pdf of having sampled this light
        static real pdf(const Scene& scene, const Light* light){
            return 1.0 / scene.lightCount();
        }
    };

}

#endif