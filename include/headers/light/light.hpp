

#ifndef PATH_TRACER_LIGHT_HPP
#define PATH_TRACER_LIGHT_HPP

#include "../core/transform.hpp"
#include "../math/vector2.hpp"
#include "lightSample.hpp"

namespace pathtracer{

    class Light{
        
    public:

        enum class LightType{
            INTERSECTABLE,
            NON_INTERSECTABLE
        };


        LightType m_type;


        Light(LightType type): m_type{type}{}


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


        LightType type() const{
            return m_type;
        }
    
    };

}

#endif