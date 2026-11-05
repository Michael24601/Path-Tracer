#ifndef PATH_TRACER_CAMERA_HPP
#define PATH_TRACER_CAMERA_HPP

#include "core/transform.hpp"
#include "math/vector2.hpp"


namespace pathtracer{

    class Ray;

    // When we need to check visibility with camera, we need direction
    // and distance (since some cameras, like the orthographic camera),
    // don't have origins.
    // This also includes info on if the point is even visible from the
    // camera, and the camera importance.
    struct CameraConnection {
        bool valid;
        Vector2 point;
        Vector3 direction;
        real distance;
        real importance;
    };


    class Camera{

    protected:

        // The width and height, in local space, of the image plane
        real m_width;
        real m_height;
        Transform m_transform;
        // Stores the inverse for faster inverse operations
        Transform m_inverse;
        
    public:

        static Transform lookAt(const Vector3& eye, 
            const Vector3& target, const Vector3& up);

        Camera(real width, real height, const Transform& transform);

        virtual Ray generateRay(const Vector2& point) const = 0;

        // Returns connection in world coordinates
        virtual CameraConnection getConnection(const Vector3& origin) const = 0;

        real width() const;

        real height() const;

    };

}

#endif