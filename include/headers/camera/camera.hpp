

#ifndef PATH_TRACER_CAMERA_HPP
#define PATH_TRACER_CAMERA_HPP

#include "../core/ray.hpp"
#include "../math/vector2.hpp"

namespace pathtracer{

    class Camera{

    public:

        virtual Ray generateRay(const Vector2& point) const = 0;

    };

}

#endif