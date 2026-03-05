
#ifndef PATH_TRACER_RAY_HPP
#define PATH_TRACER_RAY_HPP

#include "../math/vector3.hpp"

namespace pathtracer{

    class Ray{

    private:

        Vector3 m_origin;

        // Assumed to be normal
        Vector3 m_direction;

    public:

        Ray() : m_origin(Vector3::ORIGIN), m_direction(0.0, 0.0, 1.0) {};


        Ray(const Vector3& origin, const Vector3& direction) : 
            m_origin(origin), m_direction(direction) {};


        const Vector3& origin() const { return m_origin; }


        const Vector3& direction() const { return m_direction; }

        
        // Returns the point that is a distance t along the ray
        Vector3 at(real t) const {
            return m_origin + m_direction * t;
        }

    };

}

#endif