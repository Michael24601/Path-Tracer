#include "camera/camera.hpp"
#include <cassert>
#include "core/ray.hpp"
#include "math/matrix3.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"

namespace pathtracer{

    Transform Camera::lookAt(const Vector3& eye, 
        const Vector3& target, const Vector3& up) {

        Vector3 f = (target - eye).normalized();
        Vector3 r = f.cross(up).normalized();
        Vector3 u = r.cross(f);

        // NOTE: we have -f here instead of f since camera points at
        // -z by convention.
        Matrix3 rotation(r, u, -f);

        return Transform(rotation, eye);
    }

    Camera::Camera(real width, real height, const Transform& transform) :
        m_width{width}, m_height{height}, m_transform(transform) {

        assert((m_width > EPSILON && m_height > EPSILON)
            && "Camera dimensions are too small");
    }

    real Camera::width() const {
        return m_width;
    }

    real Camera::height() const {
        return m_height;
    }

}