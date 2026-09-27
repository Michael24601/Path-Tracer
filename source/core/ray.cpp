#include "core/ray.hpp"

namespace pathtracer{

    Ray::Ray() : m_origin(Vector3::ORIGIN), m_direction(0.0, 0.0, 1.0){}


    Ray::Ray(const Vector3& origin, const Vector3& direction) : 
        m_origin(origin), m_direction(direction){}


    const Vector3& Ray::origin() const {
        return m_origin;
    }


    const Vector3& Ray::direction() const {
        return m_direction;
    }


    void Ray::setOrigin(const Vector3& origin) {
        m_origin = origin;
    }


    void Ray::setDirection(const Vector3& direction) {
        m_direction = direction;
    }

    
    // Returns the point that is a distance t along the ray
    Vector3 Ray::at(real t) const {
        return m_origin + m_direction * t;
    }

}