

#ifndef PATH_TRACER_CAMERA_HPP
#define PATH_TRACER_CAMERA_HPP

#include "../core/ray.hpp"
#include "../math/vector2.hpp"
#include "../core/transform.hpp"

namespace pathtracer{

    class Camera{

    protected:

        // The width and height, in local space, of the image plane
        real m_width;
        real m_height;
        Transform m_transform;
        
    public:

        static Transform lookAt(const Vector3& eye, 
            const Vector3& target, const Vector3& up) {

            Vector3 f = (target - eye).normalized();
            Vector3 r = f.cross(up).normalized();
            Vector3 u = r.cross(f);

            Matrix3 rotation(r, u, f);

            return Transform(rotation, eye);
        }

        Camera(real width, real height, const Transform& transform) :
            m_width{width}, m_height{height}, m_transform(transform){

            assert((m_width > EPSILON && m_height > EPSILON)
                && "Camera dimensions are too small");
        }


        virtual Ray generateRay(const Vector2& point) const = 0;


        real width() const { return m_width; }


        real height() const { return m_height; }

    };

}

#endif