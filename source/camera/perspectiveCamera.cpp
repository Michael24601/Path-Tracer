#include "camera/perspectiveCamera.hpp"
#include "math/vector2.hpp"
#include "math/vector3.hpp"
#include "core/ray.hpp"

namespace pathtracer{

    PerspectiveCamera::PerspectiveCamera(real width, real height,
        const Transform& transform, real focalLength) :
        Camera(width, height, transform),
        m_focalLength{focalLength}{}


    // We assume the point ranges from -1 to 1
    Ray PerspectiveCamera::generateRay(const Vector2& point) const {
        // In local space, we always assume that the aperture
        // is at (0, 0, 0), and that the direction is -z, so that
        // the image plane center is at (0, 0, -focalLength).

        // Then we know that the given pixel will be an offset of 
        // width/2 and height/2 times the given point coordinate.
        Vector3 pixel = Vector3(
            point.x() * m_width * 0.5,
            point.y() * m_height * 0.5,
            -m_focalLength
        );

        // We then transform this point and the origin
        // so that we can generate a ray in world coordinates
        // that passes through the pixel.

        Vector3 worldPixel = m_transform.transform(pixel);
        Vector3 worldOrigin = m_transform.transform(Vector3::ORIGIN);
        Vector3 direction = (worldPixel - worldOrigin).normalized();

        return Ray(worldOrigin, direction);
    }


    Vector2 PerspectiveCamera::receiveRay(const Ray& ray) const {

        const Ray localRay = m_inverse.transform(ray);

        // Intersect with the image plane at z = -focalLength
        real t = (-m_focalLength - localRay.origin().z()) 
            / localRay.direction().z();

        if (t <= 0){
            // This means it is invalid
            return Vector2(-2, -2); 
        }

        Vector3 point = localRay.origin() + localRay.direction() * t;

        real x = point.x() / (m_width * 0.5);
        real y = point.y() / (m_height * 0.5);

        if (std::abs(x) > 1 || std::abs(y) > 1){
            return Vector2(-2, -2);
        }

        return Vector2(x, y);
    }

}