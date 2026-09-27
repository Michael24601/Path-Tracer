#ifndef PATH_TRACER_CAMERA_HPP
#define PATH_TRACER_CAMERA_HPP

#include "core/transform.hpp"

namespace pathtracer{

    class Vector2;
    class Vector3;
    class Ray;

    class Camera{

    protected:

        // The width and height, in local space, of the image plane
        real m_width;
        real m_height;
        Transform m_transform;
        
    public:

        static Transform lookAt(const Vector3& eye, 
            const Vector3& target, const Vector3& up);

        Camera(real width, real height, const Transform& transform);

        virtual Ray generateRay(const Vector2& point) const = 0;

        real width() const;

        real height() const;

    };

}

#endif