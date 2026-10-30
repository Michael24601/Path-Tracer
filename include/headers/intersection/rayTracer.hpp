
#ifndef PATH_TRACER_RAY_TRACER_HPP
#define PATH_TRACER_RAY_TRACER_HPP

#include "config.hpp"

namespace pathtracer{

    class Instance;
    class Hit;
    class Ray;

    namespace RayTracer{

        // Assuming we have a valid intersection, the alpha masking
        // function probabilistically chooses to either go through
        // or intersect the surface at this point, given the alpha
        // value at the intersection and using it as a probability.
        // Note that we input instance alongside the intersection
        // instead of using the instance parameter since it may not
        // have been set yet.

        // Note intersection is assumed to still be in local coordinates.
        // So no need to transform it for now.
        // Note that both localRay and intersection are changed.
        bool alphaMask(Hit& hit, real oldT, Ray& localRay, 
            const Instance* instance);

        // Intersects a ray with a shape instance. If we have a
        // closer intersection, we use it.
        bool intersect(Hit& oldHit, const Ray& ray, 
            const Instance* const instance);

    }

}

#endif