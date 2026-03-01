

#ifndef PATH_TRACER_CAMERA_HPP
#define PATH_TRACER_CAMERA_HPP

#include "../core/ray.hpp"
#include "../math/vector2.hpp"
#include "../core/transform.hpp"

namespace pathtracer{

    class Camera{

    protected:

        int m_width;
        int m_height;
        Transform m_transform;

    public:

        Camera(int width, int height, const Transform& transform) :
            m_width{width}, m_height{height}, m_transform(transform){}

        virtual Ray generateRay(const Vector2& point) const = 0;

    };

}

#endif