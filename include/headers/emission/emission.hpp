
#ifndef PATH_TRACER_EMISSION_HPP
#define PATH_TRACER_EMISSION_HPP

#include "config.hpp"

namespace pathtracer{

    // Forward declarations
    class Vector3;
    class Vector2;


    // Class representing the emission of an object at a specific
    // point.
    class Emission{
        
    public:

        // Note that this assumes that wo is in the shading
        // frame coordinates.
        virtual Vector3 evaluate(const Vector3& wo, const Vector2& uv)
            const = 0;

    };

}

#endif