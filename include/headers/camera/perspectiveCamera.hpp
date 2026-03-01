
#ifndef PATH_TRACER_PERSPECTIVE_CAMERA_HPP
#define PATH_TRACER_PERSPECTIVE_CAMERA_HPP

#include "camera.hpp"
#include "../math/vector2.hpp"

namespace pathtracer{

    class PerspectiveCamera : public Camera{

    private:

        real m_focalLength;

    public:


        PerspectiveCamera(int width, int height,
            const Transform& transform, real focalLength) :
            Camera(width, height, transform),
            m_focalLength{focalLength} {}


        // We assume the point ranges from -1 to 1
        Ray generateRay(const Vector2& point) const override {
            // In local space, we always assume that the aperture
            // is at (0, 0, 0), and that the direction is z, so that
            // the image plane center is at (0, 0, focalLength).
            
            // Then we know that the given pixel will be an offset of 
            // width/2 and height/2 times the given point coordinate.
            Vector3 pixel = Vector3(
                point.x() * m_width * 0.5,
                point.y() * m_height * 0.5,
                m_focalLength
            );

            // We then transform this point and the origin
            // so that we can generate a ray in world coordinates
            // that passes through the pixel.

            Vector3 worldPixel = m_transform.transform(pixel);
            Vector3 worldOrigin = m_transform.transform(Vector3::ORIGIN);
            Vector3 direction = (worldPixel - worldOrigin).normalized();

            return Ray(worldOrigin, direction);
        }

    };

}

#endif