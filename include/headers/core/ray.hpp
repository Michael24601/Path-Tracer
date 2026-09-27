
#ifndef PATH_TRACER_RAY_HPP
#define PATH_TRACER_RAY_HPP

#include "math/vector3.hpp"

namespace pathtracer{

    class Ray{

    private:

        Vector3 m_origin;

        // Assumed to be normal
        Vector3 m_direction;

    public:

        Ray();

        Ray(const Vector3& origin, const Vector3& direction);

        const Vector3& origin() const;

        const Vector3& direction() const;

        void setOrigin(const Vector3& origin);

        void setDirection(const Vector3& direction);

        
        // Returns the point that is a distance t along the ray
        Vector3 at(real t) const;

    };

}

#endif