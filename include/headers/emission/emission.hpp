
#ifndef PATH_TRACER_EMISSION_HPP
#define PATH_TRACER_EMISSION_HPP

#include "../math/vector3.hpp"
#include "../math/vector2.hpp"

namespace pathtracer{

    // Class representing the emission of an object at a specific
    // point.
    class Emission{

    protected:
        
        // The cosine term is the normal dot wo, and since we
        // are in local coordinates, the normal is the z axis.
        static real cosineTerm(const Vector3& wo) {
            return wo.z();
        }    

    public:

        // Note that this assumes that wo is in the shading
        // frame coordinates.
        virtual Vector3 evaluate(const Vector3& wo, const Vector2& uv)
            const = 0;

    };

}

#endif