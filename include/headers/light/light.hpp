

#ifndef PATH_TRACER_LIGHT_HPP
#define PATH_TRACER_LIGHT_HPP

#include "../core/transform.hpp"
#include "../math/vector2.hpp"
#include "lightSample.hpp"

namespace pathtracer{

    class Light{

    public:


        Light(){}


        // Returns the amount of light that is returned, to the
        // point, by sampling a point on the light (if sampleable.)
        // The calculation is done in global coordinates, and the
        // returned result is in world coordinates.
        // The expected input is also in world coordinates.
        virtual LightSample sample(const Vector3&) const = 0;


        // Given a particular point on the light, this returns the
        // light sample that would have been generated.
        // This only applies to intersectable lights.
        virtual LightSample evaluateLightSample(const Vector3&,
            const Vector3&) const = 0;



        // This returns true if the light is intersectable (if it is
        // an area light, and if this area light refers to an
        // instance present in the scene.)
        virtual bool isIntersectable() const = 0;
    
    };

}

#endif