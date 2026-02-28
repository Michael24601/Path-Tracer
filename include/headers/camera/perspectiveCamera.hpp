
#ifndef PATH_TRACER_PERSPECTIVE_CAMERA_HPP
#define PATH_TRACER_PERSPECTIVE_CAMERA_HPP

#include "camera.hpp"
#include "../math/vector2.hpp"

namespace pathtracer{

    class PerspectiveCamera : public Camera{

    public:

        Ray generateRay(const Vector2& point) const override {

        }

    };

}

#endif