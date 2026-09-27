

#ifndef PATH_TRACER_INTEGRATOR_HPP
#define PATH_TRACER_INTEGRATOR_HPP

namespace pathtracer{

    // Forward declarations
    class Scene;
    class Ray;
    class Vector3;

    class Integrator{

    public:
    
        // Uses whatever rendering technique it wants to render
        // the light received by the given shadow ray.
        virtual Vector3 color(const Ray&, const Scene&) = 0;
     
    };

}

#endif