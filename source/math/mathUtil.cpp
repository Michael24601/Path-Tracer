
#include "math/mathUtil.hpp"
#include <algorithm>
#include <cmath>
#include "math/vector2.hpp"
#include "math/vector2i.hpp"
#include "math/vector3.hpp"
#include "math/constants.hpp"


namespace pathtracer{


    namespace Util{

        float russianRoulette(const Vector3& c, float max) {
            return std::min(max, (float)c.luminance() * 2.0f);
        }

        int floor(real x) {
            return static_cast<int>(x);
        }

        int ceiling(real x) {
            return static_cast<int>(x + 0.5);
        }

        Vector2i floor(Vector2 uv) {
            return Vector2i(floor(uv.x()), floor(uv.y()));
        }

        Vector2i ceiling(Vector2 uv) {
            return Vector2i(ceiling(uv.x()), ceiling(uv.y()));
        }

        void swap(real& x, real& y) {
            real temp = x;
            x = y;
            y = temp;
        }

        real clamp(real value, real minVal, real maxVal) {
            return (value < minVal) ? minVal :
                (value > maxVal ? maxVal : value);
        }

    }


    namespace ShadingSpace{

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        real cosineTheta(const Vector3& w) {
            return w.z();
        }

        // The cosine term is the normal dot wi, and since we
        // are in local coordinates, the normal is the z axis.
        real absCosineTheta(const Vector3& w) {
            return std::abs(w.z());
        }

        float cosinePhiSineTheta(const Vector3& w) {
            return w.x();
        }

        float sinePhiSineTheta(const Vector3& w) {
            return w.y();
        }

        // This returns the vector reflected around the normal,
        // which in local coordinates is always (0, 0, 1).
        Vector3 reflect(const Vector3& w) {
            return Vector3(-w.x(), -w.y(), w.z());
        }

        // Reflects around a given normal
        Vector3 reflect(const Vector3& w, const Vector3& n) {
            return n * 2 * n.dot(w) - w;
        }

        // Refracts a vector given ior and normal
        Vector3 refract(const Vector3& w, const Vector3& n, real eta) {
            const real invEta = 1 / eta;
            const real k =
                1 - (invEta * invEta) *
                (1 - (n.dot(w) * n.dot(w)));

            if (k < 0) {
                // total internal reflection
                return Vector3(0.0);
            }

            const real cosTheta = n.dot(w);

            return n *
                (invEta * cosTheta -
                std::copysign(std::sqrt(k), cosTheta)) -
                w * invEta;
        }

    }


    namespace Barycentric{

        // Interpolates 3D vector
        Vector3 interpolate(const Vector3& v0, const Vector3& v1,
            const Vector3& v2, const Vector2& uv) {

            real u = uv.x();
            real v = uv.y();
            real w = 1.0 - u - v;

            return v0 * w + v1 * u + v2 * v;
        }

        // Interpolates a 2D vector
        Vector2 interpolate(const Vector2& v0, const Vector2& v1,
            const Vector2& v2, const Vector2& uv) {

            real u = uv.x();
            real v = uv.y();
            real w = 1.0 - u - v;

            return v0 * w + v1 * u + v2 * v;
        }

    }


    namespace SquareToSphereUniform{

        // Transforms uv coordinates in a unit square to 3D
        // coordinates on a unit sphere uniformly.
        Vector3 transform(const Vector2& uv) {
            real theta = 2.0 * PI * uv.x();
            real phi = std::acos(1.0 - 2.0 * uv.y());

            real sinPhi = std::sin(phi);

            return Vector3{
                sinPhi * std::cos(theta),
                sinPhi * std::sin(theta),
                std::cos(phi)
            };
        }

        // Return spherical coordinates
        Vector2 inverse(const Vector3& d) {
            real u = std::atan2(d.y(), d.x()) / (2.0 * PI);

            if (u < 0.0) {
                u += 1.0;
            }

            real v = (1.0 - d.z()) * 0.5;

            return Vector2{u, v};
        }

        // The PDF is constant since it is uniform over the surface area.
        // Note that this sampling pdf is the same for local area
        // and solid angle measures.
        real pdf(const Vector3& point) {
            return 0.25 * INV_PI;
        }

    }


    namespace SquareToHemisphereUniform{

        // Transforms uv coordinates in a unit square to 3D
        // coordinates on a unit hemisphere uniformly.
        Vector3 transform(const Vector2& uv) {
            real theta = 2.0 * PI * uv.x();

            real z = uv.y();
            real r = std::sqrt(1.0 - z * z);

            return Vector3{
                r * std::cos(theta),
                r * std::sin(theta),
                z
            };
        }

        // Transforms 3D coordinates on a unit hemisphere.
        Vector2 inverse(const Vector3& dir) {
            real theta = std::atan2(dir.y(), dir.x());

            if (theta < 0.0) {
                theta += 2.0 * PI;
            }

            real u = theta / (2.0 * PI);
            real v = dir.z();

            return Vector2{u, v};
        }

        // The PDF is constant since it is uniform over the hemisphere 
        // area. It is half of the surface area of a sphere.
        // Note that this sampling pdf is the same for local area
        // and solid angle measures.
        real pdf(const Vector3& point) {
            return 0.5 * INV_PI;
        }

    }


    // Cosine weighted
    namespace SquareToHemisphereCosine{

        Vector3 transform(const Vector2& uv) {
            real r = std::sqrt(uv.x());
            real theta = 2.0 * PI * uv.y();

            real x = r * std::cos(theta);
            real y = r * std::sin(theta);
            real z = std::sqrt(1.0 - uv.x());

            return Vector3{
                x,
                y,
                z
            };
        }

        // The point is in shading coordinates space (normal is z axis)
        real pdf(const Vector3& point) {
            // Uses absolute value in case the point given is not in
            // the upper hemisphere (can't return negative pdf, so
            // we allow it).
            return ShadingSpace::absCosineTheta(point) * INV_PI;
        }

    }

}