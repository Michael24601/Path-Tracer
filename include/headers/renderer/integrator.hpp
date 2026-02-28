

#ifndef PATH_TRACER_INTEGRATOR_HPP
#define PATH_TRACER_INTEGRATOR_HPP

#include "../core/scene.hpp"

namespace pathtracer{

    class Integrator{

    public:
    
        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        virtual Vector3 color(const Ray&, const Scene&) const = 0;
     
    };

}

#endif