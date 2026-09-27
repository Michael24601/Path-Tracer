#ifndef PATH_TRACER_PERSPECTIVE_CAMERA_HPP
#define PATH_TRACER_PERSPECTIVE_CAMERA_HPP

#include "camera/camera.hpp"

namespace pathtracer{

    class PerspectiveCamera : public Camera{

    private:

        real m_focalLength;

    public:

        PerspectiveCamera(real width, real height,
            const Transform& transform, real focalLength);

        // We assume the point ranges from -1 to 1
        Ray generateRay(const Vector2& point) const override;

    };

}

#endif